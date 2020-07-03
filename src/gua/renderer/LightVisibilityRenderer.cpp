// class header
#include <gua/renderer/LightVisibilityRenderer.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
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

    unsigned point_lights_num = 0u;
    unsigned spot_lights_num = 0u;
    unsigned sun_lights_num = 0u;
    prepare_light_table(pipe, transforms, lights, point_lights_num, spot_lights_num, sun_lights_num);
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

    draw_lights(pipe, transforms, lights, point_lights_num, spot_lights_num);

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

void LightVisibilityRenderer::prepare_light_table(Pipeline& pipe, std::vector<math::mat4>& transforms, LightTable::array_type& lights, unsigned& point_lights_num, unsigned& spot_lights_num, unsigned& sun_lights_num) const
{
    sun_lights_num = 0u;
    std::vector<math::mat4> sun_transforms;
    std::vector<LightTable::LightBlock> sun_lights;

    auto light_node_subrange = pipe.current_viewstate().scene->nodes[std::type_index(typeid(node::LightNode))];

    std::vector<node::LightNode*> light_node_pointers(light_node_subrange.size());
    std::transform(light_node_subrange.begin(), light_node_subrange.end(), light_node_pointers.begin(), [](node::Node* l) -> node::LightNode* {return reinterpret_cast<node::LightNode*>(l); } );

    // sort the nodes according to their type, such that we can use instanced draw calls in a reasonable manner
    std::sort(light_node_pointers.begin(), light_node_pointers.end(), [](node::LightNode* lhs, node::LightNode* rhs) { return lhs->data.get_type() < rhs->data.get_type();} );

    //important:
    for(auto light_node_it = light_node_pointers.begin(); light_node_it != light_node_pointers.end(); ++light_node_it) {
        auto const& light = *light_node_it;
        //auto light(l);

        LightTable::LightBlock light_block{};
        light_block.brightness = light->data.get_brightness();
        light_block.diffuse_enable = light->data.get_enable_diffuse_shading();
        light_block.specular_enable = light->data.get_enable_specular_shading();
        light_block.color = math::vec4f(light->data.get_color().vec3f().r, light->data.get_color().vec3f().g, light->data.get_color().vec3f().b, 0.f);
        light_block.type = static_cast<unsigned>(light->data.get_type());
        light_block.max_shadow_distance = light->data.get_max_shadow_dist();

        switch(light->data.get_type())
        {
        case node::LightNode::Type::POINT:
            ++point_lights_num;
            add_pointlight(pipe, *light, light_block, lights, transforms);
            break;
        case node::LightNode::Type::SPOT:
            ++spot_lights_num;
            add_spotlight(pipe, *light, light_block, lights, transforms);
            break;
        case node::LightNode::Type::SUN:
            ++sun_lights_num;
            add_sunlight(pipe, *light, light_block, sun_lights, sun_transforms);
            break;
        default:
            throw std::runtime_error("LightVisibilityRenderer::prepare_light_table(): Light type not supported.");
        };
    }
    //for(auto const& l : pipe.current_viewstate().scene->nodes[std::type_index(typeid(node::LightNode))])
    //{
    //    int i = l;
    //}



    //}

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

void LightVisibilityRenderer::draw_lights(Pipeline& pipe, std::vector<math::mat4>& transforms, LightTable::array_type& lights, unsigned const& num_point_lights, unsigned const& num_spot_lights) const
{
    auto const& ctx(pipe.get_context());
    auto gl_program(ctx.render_context->current_program());

    // proxy geometries
    auto light_sphere = std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_sphere_proxy"));
    auto light_cone = std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_cone_proxy"));

    auto& scene = *pipe.current_viewstate().scene;

    math::mat4f  view_projection_mat = math::mat4f(scene.rendering_frustum.get_projection()) * math::mat4f(scene.rendering_frustum.get_view());


    if( !(num_point_lights | num_spot_lights) ) {
        return;
    }

    ctx.render_context->bind_image(pipe.get_light_table().get_light_bitset()->get_buffer(ctx), scm::gl::FORMAT_R_32UI, scm::gl::ACCESS_READ_WRITE, 0, 0, 0);


    std::vector<math::mat4f> transforms_to_upload(transforms.begin(), transforms.begin() + num_point_lights + num_spot_lights);
    std::transform(transforms_to_upload.begin(), transforms_to_upload.end(), transforms_to_upload.begin(), [&view_projection_mat] (scm::math::mat4f const& m_transform) {return view_projection_mat * m_transform;});

    pipe.light_transform_block_.update(ctx, transforms_to_upload);
    pipe.bind_light_transformation_uniform_block(1);

    ctx.render_context->apply();
    //std::vector<uint32_t> point_light_indices;
    //std::vector<uint32_t> 

    //render point lights at once:
    if(num_point_lights > 0) {
        light_sphere->draw_instanced(pipe.get_context(), num_point_lights, 0, 0);
    }


    //std::cout << "num_spot_lights" << num_spot_lights << std::endl;
    if(num_spot_lights > 0) {
        gl_program->uniform("light_type_offset", uint(num_point_lights));
        light_cone->draw_instanced(pipe.get_context(), num_spot_lights, 0, 0);
    }
    //update offset and render spot lights at once

    //gl_program->apply_uniform(ctx, "light_type_offset",int(num_point_lights));
    //gl_program->uniform("light_type_offset", uint(num_point_lights));
    //ctx.render_context->apply();

/*
    for(unsigned int point_light_light_idx = point_light_index_range_start; point_light_light_idx < point_light_index_range_end; ++point_light_light_idx) {
        math::mat4f light_transform(transforms[point_light_light_idx]);

        auto light_mvp_mat = view_projection_mat * light_transform;

        // pre collect and use instanced draw calls with base offset
        //gl_program->uniform("gua_model_view_projection_matrix", 0, light_mvp_mat);
        gl_program->uniform("light_id", 0, int(point_light_light_idx));

        //only bind this once the moment uniforms are not uploaded individually
        //if(!is_image_bound) {
          //  is_image_bound = true;

            ctx.render_context->apply();
        //}
        // pre-assemble and use instanced draw calls
        if(lights[point_light_light_idx].type == 0) // point light
            light_sphere->draw_instanced(pipe.get_context(), 1, 0, point_light_light_idx);
    }
*/
/*
    // draw lights
    for(size_t light_index = 0; light_index < lights.size(); ++light_index)
    {
        if(lights[light_index].type == 2) // skip sun lights
            continue;



        math::mat4f light_transform(transforms[light_index]);

        auto light_mvp_mat = view_projection_mat * light_transform;

        // pre collect and use instanced draw calls with base offset
        gl_program->uniform("gua_model_view_projection_matrix", 0, light_mvp_mat);
        gl_program->uniform("light_id", 0, int(light_index));

        //only bind this once the moment uniforms are not uploaded individually
        //if(!is_image_bound) {
          //  is_image_bound = true;

            ctx.render_context->apply();
        //}
        // pre-assemble and use instanced draw calls
        if(lights[light_index].type == 0) // point light
            light_sphere->draw_instanced(pipe.get_context(), 1, 0, light_index);
        else if(lights[light_index].type == 1) // spot light
            light_cone->draw_instanced(pipe.get_context(), 1, 0, light_index);
    }
*/
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
