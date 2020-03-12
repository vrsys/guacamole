// class header
#include <gua/renderer/LightVisibilityRenderer.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/node/LightNode.hpp>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/render_device/opengl/gl_core.h>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

void LightVisibilityRenderer::render(PipelinePass& pass, Pipeline& pipe, int tile_power, unsigned ms_sample_count, bool enable_conservative, bool enable_fullscreen_fallback)
{
    auto const& ctx(pipe.get_context());
    auto const& glapi = ctx.render_context->opengl_api();
    auto const& camera = pipe.current_viewstate().camera;

    std::vector<math::mat4> transforms;
    LightTable::array_type lights;

    unsigned sun_lights_num = 0u;
    prepare_light_table(pipe, transforms, lights, sun_lights_num);
    math::vec2ui effective_resolution = pipe.get_light_table().invalidate(ctx, camera.config.get_resolution(), lights, tile_power, sun_lights_num);

    math::vec2ui rasterizer_resolution = (enable_fullscreen_fallback) ? camera.config.get_resolution() : effective_resolution;

    if(!empty_fbo_)
    {
        empty_fbo_ = ctx.render_device->create_frame_buffer();

#if 1 // workaround to avoid GL assertions
        empty_fbo_color_attachment_ = ctx.render_device->create_texture_2d(scm::math::vec2ui(rasterizer_resolution.x, rasterizer_resolution.y), scm::gl::FORMAT_RGBA_8UI);
        empty_fbo_->attach_color_buffer(0, empty_fbo_color_attachment_);
#else
        // TODO: ideally, FBOs with no attachments should be implemented in schism
        auto const& glapi = ctx.render_context->opengl_api();

        glapi.glNamedFramebufferParameteriEXT(empty_fbo_->object_id(), GL_FRAMEBUFFER_DEFAULT_WIDTH, rasterizer_resolution.x);

        glapi.glNamedFramebufferParameteriEXT(empty_fbo_->object_id(), GL_FRAMEBUFFER_DEFAULT_HEIGHT, rasterizer_resolution.y);

        glapi.glNamedFramebufferParameteriEXT(empty_fbo_->object_id(), GL_FRAMEBUFFER_DEFAULT_SAMPLES, ms_sample_count);
#endif
    }

    ctx.render_context->set_frame_buffer(empty_fbo_);

    ctx.render_context->set_viewport(scm::gl::viewport(math::vec2ui(0, 0), rasterizer_resolution));

    if(pass.depth_stencil_state())
        ctx.render_context->set_depth_stencil_state(pass.depth_stencil_state());

    if(pass.rasterizer_state())
        ctx.render_context->set_rasterizer_state(pass.rasterizer_state());

    pass.shader()->use(ctx);
    pipe.bind_light_table(pass.shader());

    if(enable_conservative)
    {
        glapi.glEnable(GL_CONSERVATIVE_RASTERIZATION_NV);
    }
    if(ms_sample_count > 0)
    {
        glapi.glEnable(GL_MULTISAMPLE);
    }

    draw_lights(pipe, transforms, lights);

    if(ms_sample_count > 0)
    {
        glapi.glDisable(GL_MULTISAMPLE);
    }
    if(enable_conservative)
    {
        glapi.glDisable(GL_CONSERVATIVE_RASTERIZATION_NV);
    }

    ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

void LightVisibilityRenderer::prepare_light_table(Pipeline& pipe, std::vector<math::mat4>& transforms, LightTable::array_type& lights, unsigned& sun_lights_num) const
{
    sun_lights_num = 0u;
    std::vector<math::mat4> sun_transforms;
    std::vector<LightTable::LightBlock> sun_lights;

    for(auto const& l : pipe.current_viewstate().scene->nodes[std::type_index(typeid(node::LightNode))])
    {
        auto light(reinterpret_cast<node::LightNode*>(l));

        LightTable::LightBlock light_block{};
        light_block.brightness = light->data.get_brightness();
        light_block.diffuse_enable = light->data.get_enable_diffuse_shading();
        light_block.specular_enable = light->data.get_enable_specular_shading();
        light_block.color = math::vec4f(light->data.get_color().vec3f().r, light->data.get_color().vec3f().g, light->data.get_color().vec3f().b, 0.f);
        light_block.type = static_cast<unsigned>(light->data.get_type());
        light_block.max_shadow_distance = light->data.get_max_shadow_dist();

        switch(light->data.get_type())
        {
        case node::LightNode::Type::SUN:
            ++sun_lights_num;
            add_sunlight(pipe, *light, light_block, sun_lights, sun_transforms);
            break;
        case node::LightNode::Type::POINT:
            add_pointlight(pipe, *light, light_block, lights, transforms);
            break;
        case node::LightNode::Type::SPOT:
            add_spotlight(pipe, *light, light_block, lights, transforms);
            break;
        default:
            throw std::runtime_error("LightVisibilityRenderer::prepare_light_table(): Light type not supported.");
        };
    }

    // sun lights need to be at the back
    transforms.insert(transforms.end(), sun_transforms.begin(), sun_transforms.end());
    lights.insert(lights.end(), sun_lights.begin(), sun_lights.end());
}

////////////////////////////////////////////////////////////////////////////////
void LightVisibilityRenderer::add_pointlight(
    Pipeline& pipe, node::LightNode& light, LightTable::LightBlock& light_block, LightTable::array_type& lights, std::vector<math::mat4>& light_transforms) const
{
    auto model_mat = light.get_cached_world_transform();
    math::vec3 light_position = model_mat * math::vec4(0.f, 0.f, 0.f, 1.f);
    float light_radius = scm::math::length(light_position - math::vec3(model_mat * math::vec4(0.f, 0.f, 1.f, 1.f)));

    light_block.position_and_radius = math::vec4f(light_position.x, light_position.y, light_position.z, light_radius);
    light_block.beam_direction_and_half_angle = math::vec4f(0.f, 0.f, 0.f, 0.f);
    light_block.falloff = light.data.get_falloff();
    light_block.softness = 0;

    if(light.data.get_enable_shadows())
    {
        if(light.data.get_max_shadow_dist() > 0.f &&
           scm::math::length(light_position - math::get_translation(pipe.current_viewstate().camera.transform)) - light_radius > light.data.get_max_shadow_dist())
        {
            light_block.casts_shadow = false;
        }
        else
        {
            light_block.casts_shadow = true;
            pipe.generate_shadow_map(light, light_block);
        }
    }

    lights.push_back(light_block);
    light_transforms.push_back(model_mat);
}

////////////////////////////////////////////////////////////////////////////////
void LightVisibilityRenderer::add_spotlight(
    Pipeline& pipe, node::LightNode& light, LightTable::LightBlock& light_block, LightTable::array_type& lights, std::vector<math::mat4>& light_transforms) const
{
    auto model_mat = light.get_cached_world_transform();

    math::vec3 light_position = model_mat * math::vec4(0.0, 0.0, 0.0, 1.0);
    math::vec3 beam_direction = math::vec3(model_mat * math::vec4(0.0, 0.0, -1.0, 1.0)) - light_position;
    float half_beam_angle = scm::math::dot(scm::math::normalize(math::vec3(model_mat * math::vec4(0.0, 0.50, -1.0, 0.0))), scm::math::normalize(beam_direction));

    light_block.position_and_radius = math::vec4f(light_position.x, light_position.y, light_position.z, 0);
    light_block.beam_direction_and_half_angle = math::vec4f(beam_direction.x, beam_direction.y, beam_direction.z, half_beam_angle);
    light_block.falloff = light.data.get_falloff();
    light_block.softness = light.data.get_softness();

    if(light.data.get_enable_shadows())
    {
        if(light.data.get_max_shadow_dist() > 0.f &&
           scm::math::length(light_position - math::get_translation(pipe.current_viewstate().camera.transform)) - scm::math::length(beam_direction) > light.data.get_max_shadow_dist())
        {
            light_block.casts_shadow = false;
        }
        else
        {
            light_block.casts_shadow = true;
            pipe.generate_shadow_map(light, light_block);
        }
    }

    lights.push_back(light_block);
    light_transforms.push_back(model_mat);
}

////////////////////////////////////////////////////////////////////////////////
void LightVisibilityRenderer::add_sunlight(
    Pipeline& pipe, node::LightNode& light, LightTable::LightBlock& light_block, LightTable::array_type& sun_lights, std::vector<math::mat4>& sunlight_transforms) const
{
    auto model_mat = light.get_cached_world_transform();
    math::vec3 light_position = model_mat * math::vec4(0.f, 0.f, 1.f, 0.f);

    light_block.position_and_radius = math::vec4f(light_position.x, light_position.y, light_position.z, 0.f);
    light_block.beam_direction_and_half_angle = math::vec4f(0.f, 0.f, 0.f, 0.f);
    light_block.falloff = 0.0f;
    light_block.softness = 0.0f;

    light_block.casts_shadow = light.data.get_enable_shadows();
    if(light.data.get_enable_shadows())
    {
        pipe.generate_shadow_map(light, light_block);
    }

    sun_lights.push_back(light_block);
    sunlight_transforms.push_back(model_mat);
}

////////////////////////////////////////////////////////////////////////////////

void LightVisibilityRenderer::draw_lights(Pipeline& pipe, std::vector<math::mat4>& transforms, LightTable::array_type& lights) const
{
    auto const& ctx(pipe.get_context());
    auto gl_program(ctx.render_context->current_program());

    // proxy geometries
    auto light_sphere = std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_sphere_proxy"));
    auto light_cone = std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_cone_proxy"));

    auto& scene = *pipe.current_viewstate().scene;

    math::mat4f  view_projection_mat = math::mat4f(scene.rendering_frustum.get_projection()) * math::mat4f(scene.rendering_frustum.get_view());

    auto camera = pipe.current_viewstate().camera;

    auto associated_window = gua::WindowDatabase::instance()->lookup(camera.config.output_window_name());//->add left_output_window
    bool is_instanced_side_by_side_enabled = false;

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        if(associated_window->config.get_stereo_mode() == StereoMode::SIDE_BY_SIDE) {
            is_instanced_side_by_side_enabled = true;
        }
#endif

math::mat4f secondary_view_projection_mat{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

if(is_instanced_side_by_side_enabled) {
    secondary_view_projection_mat = math::mat4f(scene.secondary_rendering_frustum.get_projection()) * math::mat4f(scene.secondary_rendering_frustum.get_view());
}
    // draw lights
    for(size_t i = 0; i < lights.size(); ++i)
    {
        if(lights[i].type == 2) // skip sun lights
            continue;

        math::mat4f light_transform(transforms[i]);

        auto light_mvp_mat = view_projection_mat * light_transform;
        gl_program->uniform("gua_model_view_projection_matrix", 0, light_mvp_mat);
        

if(is_instanced_side_by_side_enabled) {
        auto secondary_light_mvp_mat = secondary_view_projection_mat * light_transform;
        gl_program->uniform("gua_secondary_model_view_projection_matrix", 0, secondary_light_mvp_mat);
}

        gl_program->uniform("light_id", 0, int(i));
        ctx.render_context->bind_image(pipe.get_light_table().get_light_bitset()->get_buffer(ctx), scm::gl::FORMAT_R_32UI, scm::gl::ACCESS_READ_WRITE, 0, 0, 0);

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
if(is_instanced_side_by_side_enabled) {
        ctx.render_context->bind_image(pipe.get_light_table().get_secondary_light_bitset()->get_buffer(ctx), scm::gl::FORMAT_R_32UI, scm::gl::ACCESS_READ_WRITE, 1, 0, 0);
}
#endif //GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        ctx.render_context->apply();

        if(lights[i].type == 0) // point light
            light_sphere->draw(pipe.get_context());
        else if(lights[i].type == 1) // spot light
            light_cone->draw(pipe.get_context());
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
