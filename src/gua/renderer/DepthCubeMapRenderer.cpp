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
#include <gua/renderer/DepthCubeMapRenderer.hpp>

#include <gua/config.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/DepthCubeMap.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

#include <scm/core/math/math.h>

namespace gua
{
void DepthCubeMapRenderer::create_state_objects(RenderContext const& ctx)
{
    rs_cull_back_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);
    rs_cull_none_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);
}

void DepthCubeMapRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{
    auto& scene = *pipe.current_viewstate().scene;
    auto cube_map_nodes(scene.nodes.find(std::type_index(typeid(node::CubemapNode))));

    if(cube_map_nodes != scene.nodes.end())
    {
        RenderContext const& ctx(pipe.get_context());

        if(needs_rendering_.first != ctx.framecount)
        {
            needs_rendering_ = std::make_pair(ctx.framecount, std::vector<std::size_t>());
        }

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / DepthCubeMapPass";
        std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / DepthCubeMapPass";

        pipe.begin_gpu_query(ctx, gpu_query_name);
        pipe.begin_cpu_query(cpu_query_name);
#endif

        for(auto const& object : cube_map_nodes->second)
        {
            auto cube_map_node(reinterpret_cast<node::CubemapNode*>(object));
            bool needs_rendering = std::find(needs_rendering_.second.begin(), needs_rendering_.second.end(), cube_map_node->uuid()) == needs_rendering_.second.end();
            if(cube_map_node->config.get_active() && needs_rendering)
            {
                needs_rendering_.second.push_back(cube_map_node->uuid());
                if(cube_map_node->config.get_render_mode() == node::CubemapNode::RenderMode::COMPLETE)
                {
                    prepare_depth_cubemap(*cube_map_node, pipe);
                    generate_depth_cubemap_face(0, *cube_map_node, pipe);
                    generate_depth_cubemap_face(1, *cube_map_node, pipe);
                    generate_depth_cubemap_face(2, *cube_map_node, pipe);
                    generate_depth_cubemap_face(3, *cube_map_node, pipe);
                    generate_depth_cubemap_face(4, *cube_map_node, pipe);
                    generate_depth_cubemap_face(5, *cube_map_node, pipe);
                    download_depth_cubemap(*cube_map_node, pipe);
                }
                else if(cube_map_node->config.get_render_mode() == node::CubemapNode::RenderMode::ONE_SIDE_PER_FRAME)
                {
                    if(face_counter_ == 0)
                    {
                        prepare_depth_cubemap(*cube_map_node, pipe);
                    }

                    generate_depth_cubemap_face(face_counter_, *cube_map_node, pipe);

                    if(face_counter_ == 5)
                    {
                        download_depth_cubemap(*cube_map_node, pipe);
                    }
                }
            }
        }

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        pipe.end_gpu_query(ctx, gpu_query_name);
        pipe.end_cpu_query(cpu_query_name);
#endif
    }

    face_counter_ = (face_counter_ + 1) % 6; // cycle throgh sides
    // ctx.render_context->reset_state_objects();
}

void DepthCubeMapRenderer::prepare_depth_cubemap(node::CubemapNode const& cube_map_node, Pipeline& pipe)
{
    if(!depth_cube_map_res_)
    {
        depth_cube_map_res_ = pipe.get_context().resources.get<SharedDepthCubeMapResource>();
    }
    auto texture_name(cube_map_node.config.get_texture_name());
    std::shared_ptr<DepthCubeMap> current_depth_cube_map(nullptr);
    auto depth_cube_map_it = depth_cube_map_res_->cube_maps_.find(texture_name);

    if(depth_cube_map_it == depth_cube_map_res_->cube_maps_.end())
    {
        unsigned viewport_size(cube_map_node.config.resolution());
        unsigned map_width(viewport_size * 6);

        current_depth_cube_map = std::make_shared<DepthCubeMap>(pipe.get_context(), math::vec2ui(map_width, viewport_size), texture_name);
        ;
        depth_cube_map_res_->cube_maps_[texture_name] = current_depth_cube_map;
        current_depth_cube_map->set_viewport_size(math::vec2f(viewport_size));
    }
    else
    {
        current_depth_cube_map = depth_cube_map_it->second;
    }
    current_depth_cube_map->clear(pipe.get_context());
}

void DepthCubeMapRenderer::generate_depth_cubemap_face(unsigned face, node::CubemapNode const& cube_map_node, Pipeline& pipe) const
{
    auto depth_cube_map = depth_cube_map_res_->cube_maps_.find(cube_map_node.config.get_texture_name())->second;

    auto node_transform(cube_map_node.get_cached_world_transform());
    math::vec2ui viewport_size(depth_cube_map->get_viewport_size());

    auto orig_scene(pipe.current_viewstate().scene);
    // auto orig_shadow_mode(pipe.current_viewstate().shadow_mode);
    // auto orig_viewpoint_uuid(pipe.current_viewstate().viewpoint_uuid);

    // calculate screen transforms
    math::mat4 screen_transform(scm::math::make_translation(0., 0., -0.5));
    std::vector<math::mat4> screen_transforms({screen_transform,
                                               scm::math::make_rotation(180., 0., 1., 0.) * screen_transform,
                                               scm::math::make_rotation(90., 1., 0., 0.) * screen_transform,
                                               scm::math::make_rotation(-90., 1., 0., 0.) * screen_transform,
                                               scm::math::make_rotation(90., 0., 1., 0.) * screen_transform,
                                               scm::math::make_rotation(-90., 0., 1., 0.) * screen_transform});

    // view directions
    std::vector<PipelineViewState::ViewDirection> view_directions = {
        PipelineViewState::front, PipelineViewState::back, PipelineViewState::top, PipelineViewState::bottom, PipelineViewState::left, PipelineViewState::right};

    // frustum
    math::vec3 scale(math::get_scale(node_transform));
    math::mat4 scale_free(node_transform * scm::math::inverse(scm::math::make_scale(scale)));
    math::mat4 transform(scale_free * screen_transforms[face]);
    auto frustum = Frustum::perspective(scale_free, transform, cube_map_node.config.near_clip(), cube_map_node.config.far_clip());

    depth_cube_map->set_viewport_offset(math::vec2f(face, 0.f));

    // create two copies of view state, one for cubemap rendering, one for reseting the old configuration
    PipelineViewState new_view_state(pipe.current_viewstate());
    PipelineViewState old_view_state(pipe.current_viewstate());

    new_view_state.target = depth_cube_map.get();
    new_view_state.shadow_mode = true;
    new_view_state.view_direction = view_directions[face];
    new_view_state.viewpoint_uuid = cube_map_node.uuid();

    new_view_state.scene = new_view_state.graph->serialize(frustum,
                                                           frustum,
                                                           math::get_translation(node_transform),
                                                           true, // Frustum Culling
                                                           cube_map_node.config.mask(),
                                                           cube_map_node.config.view_id());
    new_view_state.frustum = frustum;

    pipe.camera_block_.update(pipe.get_context(), frustum, frustum.get_camera_position(), new_view_state.scene->clipping_planes, cube_map_node.config.view_id(), viewport_size);
    pipe.bind_camera_uniform_block(0);

    pipe.current_viewstate_ = new_view_state;

    // process all passes
    for(uint32_t pass_idx = 0; pass_idx < pipe.passes_.size(); ++pass_idx)
    {
        if(pipe.passes_[pass_idx].enable_for_shadows())
        {
            pipe.passes_[pass_idx].process(*pipe.last_description_.get_passes()[pass_idx], pipe, false, false);
        }
    }

    // restore previous configuration
    pipe.current_viewstate_ = old_view_state;
    pipe.camera_block_.update(pipe.get_context(),
                              old_view_state.scene->rendering_frustum,
                              math::get_translation(old_view_state.camera.transform),
                              old_view_state.scene->clipping_planes,
                              old_view_state.camera.config.get_view_id(),
                              old_view_state.camera.config.get_resolution());
    pipe.bind_camera_uniform_block(0);
}

void DepthCubeMapRenderer::download_depth_cubemap(node::CubemapNode& cube_map_node, Pipeline const& pipe) const
{
    auto texture_name(cube_map_node.config.get_texture_name());
    std::shared_ptr<DepthCubeMap> current_depth_cube_map(nullptr);
    auto depth_cube_map_it = depth_cube_map_res_->cube_maps_.find(texture_name);

    if(depth_cube_map_it != depth_cube_map_res_->cube_maps_.end())
    {
        current_depth_cube_map = depth_cube_map_it->second;
        current_depth_cube_map->retrieve_data(pipe.get_context(), cube_map_node.config.near_clip(), cube_map_node.config.far_clip());
        *(cube_map_node.m_NewTextureData) = true;
    }
}

} // namespace gua
