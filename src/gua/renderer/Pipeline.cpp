/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/renderer/Pipeline.hpp>

// guacamole headers
#include <gua/config.hpp>
#include <gua/node/CameraNode.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/WindowBase.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/node/CameraNode.hpp>
#include <gua/node/LightNode.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/renderer/LightTable.hpp>

// external headers
#include <iostream>

namespace
{
gua::math::vec2ui get_handle(scm::gl::texture_image_ptr const& tex)
{
    uint64_t handle = tex->native_handle();
    return gua::math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);
}

} // namespace

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

Pipeline::Pipeline(RenderContext& ctx, math::vec2ui const& resolution)
    : current_viewstate_(), context_(ctx), gbuffer_(new GBuffer(ctx, resolution)), camera_block_(ctx.render_device), light_table_(new LightTable), last_resolution_(0, 0), last_description_(),
      global_substitution_map_(), passes_(), responsibilities_pre_render_(), responsibilities_post_render_(),
      quad_(new scm::gl::quad_geometry(ctx.render_device, scm::math::vec2f(-1.f, -1.f), scm::math::vec2f(1.f, 1.f))),
      box_(new scm::gl::box_geometry(ctx.render_device, scm::math::vec3f(0.f, 0.f, 0.f), scm::math::vec3f(1.f, 1.f, 1.f)))
{
    const float th = last_description_.get_blending_termination_threshold();
    global_substitution_map_["enable_abuffer"] = "0";

    global_substitution_map_["abuf_insertion_threshold"] = std::to_string(th);
    global_substitution_map_["abuf_blending_termination_threshold"] = std::to_string(th);
    global_substitution_map_["max_lights_num"] = std::to_string(last_description_.get_max_lights_count());

    load_passes_and_responsibilities();
}

void Pipeline::load_passes_and_responsibilities()
{
    std::vector<std::shared_ptr<PipelineResponsibilityDescription>> responsibility_descriptions;
    std::set<std::string> responsibility_names;

    for(const auto& pass_desc : last_description_.get_passes())
    {
        passes_.push_back(pass_desc->make_pass(context_, global_substitution_map_));

        for(const auto& pass_responsibility : pass_desc->get_responsibilities())
        {
            if (responsibility_names.find(pass_responsibility->private_.name_) == responsibility_names.cend()){
                responsibility_names.insert(pass_responsibility->private_.name_);
                responsibility_descriptions.push_back(pass_responsibility);
            }
        }
    }

    for(auto& responsibility_description : responsibility_descriptions)
    {
        auto responsibility = responsibility_description->make_responsibility(*this);
        if(responsibility.type() == PipelineResponsibilityPrivate::TYPE::PRE_RENDER)
        {
            responsibilities_pre_render_.push_back(responsibility);
        }
        if(responsibility.type() == PipelineResponsibilityPrivate::TYPE::POST_RENDER)
        {
            responsibilities_post_render_.push_back(responsibility);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

scm::gl::texture_2d_ptr Pipeline::render_scene(CameraMode mode, node::SerializedCameraNode const& original_camera, std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs)
{
	node::SerializedCameraNode camera(original_camera);

    // return if pipeline is disabled
    if(!camera.config.get_enabled())
    {
        return {nullptr};
    }

	bool rendering_for_hmd = false;
    if(camera.camera_node_name.find("Vive-HMD-User") != std::string::npos)
    {
        rendering_for_hmd = true;
		auto camera_transform = context_.render_window->get_latest_matrices(4);
        camera.transform = camera.parents_transform * camera_transform;
        node::SerializedCameraNode::camera_nodes[camera.uuid]->set_transform(camera_transform);
    }

    // store the current camera data
    current_viewstate_.camera = camera;
    current_viewstate_.viewpoint_uuid = camera.uuid;
    current_viewstate_.view_direction = PipelineViewState::front;
    current_viewstate_.shadow_mode = false;

    bool reload_gbuffer(false);
    bool reload_abuffer(false);

    // execute all prerender cameras
    for(auto const& cam : camera.pre_render_cameras)
    {
        if(!context_.render_pipelines.count(cam.uuid))
        {
            context_.render_pipelines.insert(std::make_pair(cam.uuid, std::make_shared<Pipeline>(context_, camera.config.get_resolution())));
        }
        context_.render_pipelines[cam.uuid]->render_scene(mode, cam, scene_graphs);
    }

    // recreate gbuffer if resolution changed

    auto adjusted_camera_resolution = camera.config.get_resolution();

    #ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        std::cout << "Ist an" << std::endl;
        adjusted_camera_resolution.x *= 2;
    #endif

    if(last_resolution_ != adjusted_camera_resolution)
    {
        last_resolution_ = adjusted_camera_resolution;
        reload_gbuffer = true;
    }

    if(reload_gbuffer)
    {
        if(gbuffer_)
        {
            gbuffer_->remove_buffers(get_context());
        }

        math::vec2ui new_gbuf_size(std::max(1U, adjusted_camera_resolution.x), std::max(1U, adjusted_camera_resolution.y));
        gbuffer_.reset(new GBuffer(get_context(), new_gbuf_size));
    }

    // recreate pipeline passes if pipeline description changed
    bool reload_passes(reload_gbuffer);

    if(*camera.pipeline_description != last_description_)
    {
        reload_passes = true;
        reload_abuffer = true;
        last_description_ = *camera.pipeline_description;
    }
    else
    {
        // if pipeline configuration is unchanged, update only uniforms of passes
        for(unsigned i(0); i < last_description_.get_passes().size(); ++i)
        {
            last_description_.get_passes()[i]->uniforms = camera.pipeline_description->get_passes()[i]->uniforms;
        }
    }

    if(reload_abuffer)
    {
        gbuffer_->allocate_a_buffer(context_, last_description_.get_enable_abuffer() ? last_description_.get_abuffer_size() : 0);
    }

    if(reload_passes)
    {
        for(auto& pass : passes_)
        {
            pass.on_delete(this);
        }

        passes_.clear();
        global_substitution_map_.clear();
        responsibilities_pre_render_.clear();
        responsibilities_post_render_.clear();

        const float th = last_description_.get_blending_termination_threshold();
        global_substitution_map_["enable_abuffer"] = last_description_.get_enable_abuffer() ? "1" : "0";


#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        if (camera.config.get_enable_stereo())
        {
            global_substitution_map_["get_enable_multi_view_rendering"] = "1";
        } else {
            global_substitution_map_["get_enable_multi_view_rendering"] = "0";
        }

#else
        global_substitution_map_["get_enable_multi_view_rendering"] = "0";

#endif //GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING


        global_substitution_map_["abuf_insertion_threshold"] = std::to_string(th);
        global_substitution_map_["abuf_blending_termination_threshold"] = std::to_string(th);
        global_substitution_map_["max_lights_num"] = std::to_string(last_description_.get_max_lights_count());

        load_passes_and_responsibilities();
    }

    // get scenegraph which shall be rendered
    current_viewstate_.graph = nullptr;
    for(auto& graph : scene_graphs)
    {
        if(graph->get_name() == camera.config.get_scene_graph_name())
        {
            current_viewstate_.graph = graph.get();
            break;
        }
    }

    context_.mode = mode;

    // serialize this scenegraph
    current_viewstate_.scene = current_viewstate_.graph->serialize(camera, mode);
    current_viewstate_.frustum = current_viewstate_.scene->rendering_frustum;

    if(rendering_for_hmd)
    {
        camera_block_.updateHMD(context_, current_viewstate_.scene->rendering_frustum, camera.parents_transform, math::get_translation(camera.transform), current_viewstate_.scene->clipping_planes,
                                camera.config.get_view_id(),
                             camera.config.get_resolution());
    }
    else
    {
#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        camera_block_.update(context_, 
                             current_viewstate_.scene->rendering_frustum, 
                             current_viewstate_.scene->secondary_rendering_frustum,
                             math::get_translation(camera.transform), 
                             current_viewstate_.scene->clipping_planes, 
                             camera.config.get_view_id(),
                             camera.config.get_resolution());
            bind_camera_uniform_block(0);

#else
        camera_block_.update(context_, 
                             current_viewstate_.scene->rendering_frustum, 
                             math::get_translation(camera.transform), 
                             current_viewstate_.scene->clipping_planes, 
                             camera.config.get_view_id(),
                             camera.config.get_resolution());
            bind_camera_uniform_block(0);


#endif // GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
    }


    // clear gbuffer
    gbuffer_->clear(context_, 1.f, 1);
    current_viewstate_.target = gbuffer_.get();

    // process all passes
    for(unsigned i(0); i < passes_.size(); ++i)
    {
        if(passes_[i].needs_color_buffer_as_input())
        {
            gbuffer_->toggle_ping_pong();
        }
        passes_[i].process(*last_description_.get_passes()[i], *this);
    }

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    fetch_gpu_query_results(context_);

    if(context_.framecount % 60 == 0)
    {
        std::cout << "===== Time Queries for Context: " << context_.id << " ============================" << std::endl;
        for(auto const& t : queries_.results)
        {
            std::cout << t.first << " : " << t.second << " ms" << std::endl;
        }
        queries_.results.clear();
        std::cout << ">>====================================================" << std::endl;
    }
#endif

    gbuffer_->toggle_ping_pong();

    // add texture to texture database
    auto tex_name(camera.config.get_output_texture_name());
    scm::gl::texture_2d_ptr color_tex = gbuffer_->get_color_buffer();

    if(tex_name != "")
    {
        if(auto dummy_tex = TextureDatabase::instance()->lookup(tex_name))
        {
            context_.textures[dummy_tex->uuid()] = RenderContext::Texture{color_tex, gbuffer_->get_sampler_state()};
            // context_.textures[dummy_tex->uuid()] = RenderContext::Texture{gbuffer_->get_color_buffer(), gbuffer_->get_sampler_state()};
        }

        if(auto dummy_tex = TextureDatabase::instance()->lookup(tex_name + "_depth"))
        {
            scm::gl::texture_2d_ptr depth_tex = gbuffer_->get_depth_buffer();
            context_.textures[dummy_tex->uuid()] = RenderContext::Texture{depth_tex, gbuffer_->get_sampler_state()};
        }
    }

    return color_tex;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::fulfil_pre_render_responsibilities(RenderContext const& ctx)
{
    for(auto& responsibility : responsibilities_pre_render_)
    {
        responsibility.fulfil(*this);
    }
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::fulfil_post_render_responsibilities(RenderContext const& ctx)
{
    for(auto& responsibility : responsibilities_post_render_)
    {
        responsibility.fulfil(*this);
    }
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::generate_shadow_map(node::LightNode& light, LightTable::LightBlock& light_block)
{
    if(!shadow_map_res_)
    {
        shadow_map_res_ = context_.resources.get<SharedShadowMapResource>();
    }

    auto& current_mask(current_viewstate_.camera.config.mask());

    std::shared_ptr<ShadowMap> shadow_map(nullptr);
    bool needs_redraw(false);

    if(light.data.get_type() == node::LightNode::Type::SUN)
    {
        // cascaded shadow maps need to be redrawn every frame
        needs_redraw = true;
    }

    // has the shadow map been rendered this frame already?
    auto cached_shadow_maps(shadow_map_res_->used_shadow_maps.find(&light));

    if(cached_shadow_maps != shadow_map_res_->used_shadow_maps.end())
    {
        for(auto& cached_shadow_map : cached_shadow_maps->second)
        {
            if(cached_shadow_map.render_mask == current_mask)
            {
                shadow_map = cached_shadow_map.shadow_map;
                break;
            }
        }
    }

    unsigned viewport_size(light.data.shadow_map_size());
    unsigned cascade_count(1);

    if(light.data.get_type() == node::LightNode::Type::SUN)
    {
        cascade_count = light.data.get_shadow_cascaded_splits().size() - 1;
    }
    else if(light.data.get_type() == node::LightNode::Type::POINT)
    {
        cascade_count = 6;
    }

    // cascades are aligned horizontally on the shadow map
    unsigned map_width(viewport_size * cascade_count);

    light_block.cascade_count = cascade_count;

    // if shadow map hasn't been rendered yet, try to find an unused one
    if(!shadow_map)
    {
        needs_redraw = true;
        for(auto it(shadow_map_res_->unused_shadow_maps.begin()); it != shadow_map_res_->unused_shadow_maps.end(); ++it)
        {
            if((*it)->get_width() == map_width)
            {
                shadow_map = *it;
                shadow_map_res_->unused_shadow_maps.erase(it);
                break;
            }
        }
    }

    // if there is still no shadow map, create a new one
    if(!shadow_map)
    {
        shadow_map = std::make_shared<ShadowMap>(context_, math::vec2ui(map_width, viewport_size));
    }

    // store shadow map
    shadow_map_res_->used_shadow_maps[&light].push_back({shadow_map, current_mask});

    current_viewstate_.target = shadow_map.get();

    auto orig_scene(current_viewstate_.scene);

    // clear shadow map
    if(needs_redraw)
    {
        shadow_map->clear(context_);
        shadow_map->set_viewport_size(math::vec2f(viewport_size));
    }

    // set view parameters
    auto original_camera_resolution = current_viewstate_.camera.config.get_resolution();
    auto shadow_resolution = gua::math::vec2ui{viewport_size, viewport_size};

    current_viewstate_.viewpoint_uuid = light.uuid();
    current_viewstate_.view_direction = PipelineViewState::front;
    current_viewstate_.shadow_mode = true;
    current_viewstate_.camera.config.set_resolution(shadow_resolution);

    // render cascaded shadows for sunlights
    switch(light.data.get_type())
    {
    case node::LightNode::Type::SUN:
        generate_shadow_map_sunlight(shadow_map, light, light_block, viewport_size, needs_redraw, orig_scene->rendering_frustum.get_screen_transform());
        break;
    case node::LightNode::Type::POINT:
        generate_shadow_map_pointlight(shadow_map, light, light_block, viewport_size, needs_redraw);
        break;
    case node::LightNode::Type::SPOT:
        generate_shadow_map_spotlight(light, light_block, viewport_size, needs_redraw);
        break;
    default:
        throw std::runtime_error("Pipeline::generate_shadow_map(): Lightnode type not supported.");
    }

    // restore previous configuration
    current_viewstate_.target = gbuffer_.get();
    current_viewstate_.scene = orig_scene;
    current_viewstate_.frustum = current_viewstate_.scene->rendering_frustum;
    current_viewstate_.camera.config.set_resolution(original_camera_resolution);

    camera_block_.update(context_,
                         current_viewstate_.scene->rendering_frustum,
                         math::get_translation(current_viewstate_.camera.transform),
                         current_viewstate_.scene->clipping_planes,
                         current_viewstate_.camera.config.get_view_id(),
                         current_viewstate_.camera.config.get_resolution());

    bind_camera_uniform_block(0);

    light_block.shadow_offset = light.data.get_shadow_offset();
    light_block.shadow_map = ::get_handle(shadow_map->get_depth_buffer());
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::render_shadow_map(LightTable::LightBlock& light_block, Frustum const& frustum, unsigned cascade_id, unsigned viewport_size, bool redraw)
{
    light_block.projection_view_mats[cascade_id] = math::mat4f(frustum.get_projection() * frustum.get_view());

    // only render shadow map if it hasn't been rendered before this frame
    if(redraw)
    {
        current_viewstate_.scene = current_viewstate_.graph->serialize(frustum,
                                                                       frustum,
                                                                       math::get_translation(current_viewstate_.camera.transform),
                                                                       current_viewstate_.camera.config.enable_frustum_culling(),
                                                                       current_viewstate_.camera.config.mask(),
                                                                       current_viewstate_.camera.config.view_id());

        current_viewstate_.frustum = frustum;

        camera_block_.update(context_, frustum, frustum.get_camera_position(), current_viewstate_.scene->clipping_planes, current_viewstate_.camera.config.get_view_id(), math::vec2ui(viewport_size));
        bind_camera_uniform_block(0);

        // process all passes
        for(std::size_t pass_idx = 0; pass_idx < passes_.size(); ++pass_idx)
        {
            if(passes_[pass_idx].enable_for_shadows())
            {
                passes_[pass_idx].process(*last_description_.get_passes()[pass_idx], *this);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
void Pipeline::generate_shadow_map_sunlight(
    std::shared_ptr<ShadowMap> const& shadow_map, node::LightNode& light, LightTable::LightBlock& light_block, unsigned viewport_size, bool redraw, math::mat4 const& original_screen_transform)
{
    auto splits(light.data.get_shadow_cascaded_splits());

    if(light.data.get_shadow_cascaded_splits().size() < 2)
    {
        Logger::LOG_WARNING << "At least 2 splits have to be defined for cascaded shadow maps!" << std::endl;
    }

    if(current_viewstate_.camera.config.near_clip() > splits.front() || current_viewstate_.camera.config.far_clip() < splits.back())
    {
        Logger::LOG_WARNING << "Splits of cascaded shadow maps are not inside "
                            << "clipping range! Fallback to equidistant splits used." << std::endl;

        float clipping_range(current_viewstate_.camera.config.far_clip() - current_viewstate_.camera.config.near_clip());
        splits = {current_viewstate_.camera.config.near_clip(),
                  current_viewstate_.camera.config.near_clip() + clipping_range * 0.25f,
                  current_viewstate_.camera.config.near_clip() + clipping_range * 0.5f,
                  current_viewstate_.camera.config.near_clip() + clipping_range * 0.75f,
                  current_viewstate_.camera.config.far_clip()};
    }

    for(uint32_t cascade = 0; cascade < splits.size() - 1; ++cascade)
    {
        shadow_map->set_viewport_offset(math::vec2f(cascade, 0.f));

        // set clipping of camera frustum according to current cascade
        // use cyclops for consistent cascades for left and right eye in stereo
        Frustum cropped_frustum(Frustum::perspective(
            // orig_scene->rendering_frustum.get_camera_transform(),
            current_viewstate_.camera.transform,
            original_screen_transform,
            splits[cascade],
            splits[cascade + 1]));

        // transform cropped frustum into sun space and calculate radius and
        // bbox of transformed frustum
        auto cropped_frustum_corners(cropped_frustum.get_corners());
        math::BoundingBox<math::vec3> extends_in_sun_space;
        float radius_in_sun_space = 0;
        std::vector<math::vec3> corners_in_sun_space;
        math::vec3 center_in_sun_space(0, 0, 0);

        auto transform(light.get_cached_world_transform());

        auto inverse_sun_transform(scm::math::inverse(transform));
        for(auto const& corner : cropped_frustum_corners)
        {
            math::vec3 new_corner(inverse_sun_transform * corner);
            center_in_sun_space += new_corner / 8;
            corners_in_sun_space.push_back(new_corner);
            extends_in_sun_space.expandBy(new_corner);
        }

        for(auto const& corner : corners_in_sun_space)
        {
            float radius = scm::math::length(corner - center_in_sun_space);
            if(radius > radius_in_sun_space)
            {
                radius_in_sun_space = radius;
            }
        }

        // center of front plane of frustum
        auto center(math::vec3f((extends_in_sun_space.min[0] + extends_in_sun_space.max[0]) / 2,
                                (extends_in_sun_space.min[1] + extends_in_sun_space.max[1]) / 2,
                                extends_in_sun_space.max[2] + light.data.get_shadow_near_clipping_in_sun_direction()));

        // eliminate sub-pixel movement
        float tex_coord_x = center.x * viewport_size / radius_in_sun_space / 2;
        float tex_coord_y = center.y * viewport_size / radius_in_sun_space / 2;

        float tex_coord_rounded_x = round(tex_coord_x);
        float tex_coord_rounded_y = round(tex_coord_y);

        float dx = tex_coord_rounded_x - tex_coord_x;
        float dy = tex_coord_rounded_y - tex_coord_y;

        dx /= viewport_size / radius_in_sun_space / 2;
        dy /= viewport_size / radius_in_sun_space / 2;

        center.x += dx;
        center.y += dy;

        // calculate transformation of shadow screen
        math::mat4 screen_in_sun_space(scm::math::make_translation(center) * scm::math::make_scale(radius_in_sun_space * 2, radius_in_sun_space * 2, 1.0f));
        auto sun_screen_transform(transform * screen_in_sun_space);

        // calculate transformation of shadow eye
        auto sun_eye_transform(scm::math::make_translation(sun_screen_transform.column(3)[0], sun_screen_transform.column(3)[1], sun_screen_transform.column(3)[2]));
        auto sun_eye_depth(transform * math::vec4(0, 0, extends_in_sun_space.max[2] - extends_in_sun_space.min[2] + light.data.get_shadow_near_clipping_in_sun_direction(), 0.0f));

        auto shadow_frustum(Frustum::orthographic(sun_eye_transform, sun_screen_transform, 0, scm::math::length(sun_eye_depth) + light.data.get_shadow_far_clipping_in_sun_direction()));

        render_shadow_map(light_block, shadow_frustum, cascade, viewport_size, redraw);
    }
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::generate_shadow_map_pointlight(std::shared_ptr<ShadowMap> const& shadow_map, node::LightNode& light, LightTable::LightBlock& light_block, unsigned viewport_size, bool redraw)
{
    // calculate light frustum
    math::mat4 screen_transform(scm::math::make_translation(0., 0., -0.5));

    std::vector<math::mat4> screen_transforms = {screen_transform,
                                                 scm::math::make_rotation(180., 0., 1., 0.) * screen_transform,
                                                 scm::math::make_rotation(90., 1., 0., 0.) * screen_transform,
                                                 scm::math::make_rotation(-90., 1., 0., 0.) * screen_transform,
                                                 scm::math::make_rotation(90., 0., 1., 0.) * screen_transform,
                                                 scm::math::make_rotation(-90., 0., 1., 0.) * screen_transform};

    std::vector<PipelineViewState::ViewDirection> view_directions = {
        PipelineViewState::front, PipelineViewState::back, PipelineViewState::top, PipelineViewState::bottom, PipelineViewState::left, PipelineViewState::right};

    for(unsigned cascade(0); cascade < screen_transforms.size(); ++cascade)
    {
        auto transform(light.get_cached_world_transform() * screen_transforms[cascade]);

        // TODO: consider light scale for clipping planes?
        auto light_near_clip = light.data.get_shadow_near_clipping_in_sun_direction();
        auto light_far_clip = light.data.get_shadow_far_clipping_in_sun_direction();

        auto frustum(Frustum::perspective(light.get_cached_world_transform(), transform, light_near_clip, light_far_clip));
        shadow_map->set_viewport_offset(math::vec2f(cascade, 0.0));

        current_viewstate_.view_direction = view_directions[cascade];

        render_shadow_map(light_block, frustum, cascade, viewport_size, redraw);
    }
}

////////////////////////////////////////////////////////////////////////////////
void Pipeline::generate_shadow_map_spotlight(node::LightNode& light, LightTable::LightBlock& light_block, unsigned viewport_size, bool redraw)
{
    // calculate light frustum
    math::mat4 screen_transform(scm::math::make_translation(0., 0., -1.));
    screen_transform = light.get_cached_world_transform() * screen_transform;

    auto light_near_clip = light.data.get_shadow_near_clipping_in_sun_direction();
    auto light_far_clip = light.data.get_shadow_far_clipping_in_sun_direction();

    auto frustum(Frustum::perspective(light.get_cached_world_transform(), screen_transform, light_near_clip, light_far_clip));

    render_shadow_map(light_block, frustum, 0, viewport_size, redraw);
}

////////////////////////////////////////////////////////////////////////////////

PipelineViewState const& Pipeline::current_viewstate() const { return current_viewstate_; }

////////////////////////////////////////////////////////////////////////////////

RenderContext& Pipeline::get_context() { return context_; }

////////////////////////////////////////////////////////////////////////////////

RenderContext const& Pipeline::get_context() const { return context_; }

////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<GBuffer> const& Pipeline::get_gbuffer() const { return gbuffer_; }

////////////////////////////////////////////////////////////////////////////////

LightTable& Pipeline::get_light_table() { return *light_table_; }

////////////////////////////////////////////////////////////////////////////////

void Pipeline::bind_gbuffer_input(std::shared_ptr<ShaderProgram> const& shader) const
{
    shader->set_uniform(context_, 1.0f / gbuffer_->get_width(), "gua_texel_width");
    shader->set_uniform(context_, 1.0f / gbuffer_->get_height(), "gua_texel_height");

    shader->set_uniform(context_, math::vec2ui(gbuffer_->get_width(), gbuffer_->get_height()), "gua_resolution");

    shader->set_uniform(context_, ::get_handle(gbuffer_->get_color_buffer()), "gua_gbuffer_color");
    shader->set_uniform(context_, ::get_handle(gbuffer_->get_pbr_buffer()), "gua_gbuffer_pbr");
    shader->set_uniform(context_, ::get_handle(gbuffer_->get_normal_buffer()), "gua_gbuffer_normal");
    shader->set_uniform(context_, ::get_handle(gbuffer_->get_flags_buffer()), "gua_gbuffer_flags");
    shader->set_uniform(context_, ::get_handle(gbuffer_->get_depth_buffer()), "gua_gbuffer_depth");

}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::bind_light_table(std::shared_ptr<ShaderProgram> const& shader) const
{
    shader->set_uniform(context_, int(light_table_->get_lights_num()), "gua_lights_num");
    shader->set_uniform(context_, int(light_table_->get_sun_lights_num()), "gua_sun_lights_num");

    if(light_table_->get_light_bitset() && light_table_->get_lights_num() > 0)
    {
        shader->set_uniform(context_, light_table_->get_light_bitset()->get_handle(context_), "gua_light_bitset");
        context_.render_context->bind_uniform_buffer(light_table_->light_uniform_block().block_buffer(), 1);
    }
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::bind_camera_uniform_block(unsigned location) const { get_context().render_context->bind_uniform_buffer(camera_block_.block().block_buffer(), location); }

////////////////////////////////////////////////////////////////////////////////

void Pipeline::draw_quad() { quad_->draw(context_.render_context); }

////////////////////////////////////////////////////////////////////////////////

void Pipeline::draw_quad_instanced(const int in_instance_count) { quad_->draw_instanced(context_.render_context, in_instance_count); }

////////////////////////////////////////////////////////////////////////////////

void Pipeline::draw_box() { box_->draw(context_.render_context); }

////////////////////////////////////////////////////////////////////////////////
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
void Pipeline::begin_cpu_query(std::string const& query_name)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    queries_.cpu_queries[query_name] = start_time;
}
#endif

////////////////////////////////////////////////////////////////////////////////
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
void Pipeline::end_cpu_query(std::string const& query_name)
{
    assert(queries_.cpu_queries.count(query_name));

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point start_time = queries_.cpu_queries.at(query_name);

    double mcs = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    queries_.results[query_name] = mcs / 1000.0;
}
#endif

////////////////////////////////////////////////////////////////////////////////
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
void Pipeline::begin_gpu_query(RenderContext const& ctx, std::string const& name)
{
    if(ctx.framecount < 50)
    {
        queries_.gpu_queries.clear();
        return;
    }

    auto existing_query = queries_.gpu_queries.find(name);

    if(existing_query != queries_.gpu_queries.end())
    {
        // delete existing query if it is too old
        const unsigned max_wait_frames = 50;
        if(existing_query->second.collect_attempts > max_wait_frames)
        {
            queries_.gpu_queries.erase(existing_query);
        }
        else
        {
            // existing query in process -> nothing to be done!!! -> return!!
            return;
        }
    }

    try
    {
        // create query
        auto query = ctx.render_device->create_timer_query();
        query_dispatch dispatch = {query, false, 0U};

        queries_.gpu_queries.insert(std::make_pair(name, dispatch));
        ctx.render_context->begin_query(query);
    }
    catch(...)
    {
        // query dispatch failed
    }
}
#endif

////////////////////////////////////////////////////////////////////////////////
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
void Pipeline::end_gpu_query(RenderContext const& ctx, std::string const& name)
{
    // query started
    if(queries_.gpu_queries.count(name))
    {
        // query not finished yet
        if(!queries_.gpu_queries.at(name).dispatched)
        {
            ctx.render_context->end_query(queries_.gpu_queries.at(name).query);
            queries_.gpu_queries.at(name).dispatched = true;
        }
        else
        {
            // no such query
            return;
        }
    }
}
#endif

////////////////////////////////////////////////////////////////////////////////
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
void Pipeline::fetch_gpu_query_results(RenderContext const& ctx)
{
    bool queries_ready = true;
    for(auto& q : queries_.gpu_queries)
    {
        bool query_ready = ctx.render_context->query_result_available(q.second.query);
        ++q.second.collect_attempts;
        queries_ready &= query_ready;
    }

    if(queries_ready)
    {
        for(auto const& q : queries_.gpu_queries)
        {
            ctx.render_context->collect_query_results(q.second.query);
            double draw_time_in_ms = static_cast<double>(q.second.query->result()) / 1e6;
            queries_.results[q.first] = draw_time_in_ms;
        }

        queries_.gpu_queries.clear();
    }
}
#endif

////////////////////////////////////////////////////////////////////////////////

void Pipeline::clear_frame_cache()
{
    if(!shadow_map_res_)
    {
        shadow_map_res_ = context_.resources.get<SharedShadowMapResource>();
    }

    for(auto& unused : shadow_map_res_->unused_shadow_maps)
    {
        unused->unbind(context_);
        unused->remove_buffers(context_);
    }
    shadow_map_res_->unused_shadow_maps.clear();

    for(auto& cached_maps : shadow_map_res_->used_shadow_maps)
    {
        for(auto& cached_map : cached_maps.second)
        {
            shadow_map_res_->unused_shadow_maps.insert(cached_map.shadow_map);
        }
    }

    shadow_map_res_->used_shadow_maps.clear();
}

////////////////////////////////////////////////////////////////////////////////



} // namespace gua
