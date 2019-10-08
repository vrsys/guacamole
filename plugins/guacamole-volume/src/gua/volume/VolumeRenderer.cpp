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
#include <gua/volume/VolumeRenderer.hpp>

#include <gua/config.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/volume/Volume.hpp>
#include <gua/volume/VolumeNode.hpp>
#include <gua/renderer/GBuffer.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

VolumeRenderer::VolumeRenderer()
    : program_factory_(), volume_raygeneration_fbo_(nullptr), volume_raygeneration_color_buffer_(), volume_raygeneration_depth_buffer_(), composite_shader_(), ray_generation_shader_(),
      depth_stencil_state_(nullptr), blend_state_(nullptr)
{
}

////////////////////////////////////////////////////////////////////////////////

void VolumeRenderer::render(Pipeline& pipe)
{
    auto& scene = *pipe.current_viewstate().scene;
    auto& target = *pipe.current_viewstate().target;

    if(scene.nodes[std::type_index(typeid(node::VolumeNode))].size() > 0)
    {
        if(!volume_raygeneration_fbo_)
        {
            init_resources(pipe);
        }

        auto const& ctx(pipe.get_context());

        ctx.render_context->set_depth_stencil_state(depth_stencil_state_, 1);

        // 1. render proxy geometry into fbo
        ctx.render_context->set_frame_buffer(volume_raygeneration_fbo_);
        {
            ctx.render_context->set_viewport(scm::gl::viewport(math::vec2f(0.0f, 0.0f), math::vec2f(float(target.get_width()), float(target.get_height()))));
            ctx.render_context->clear_color_buffers(volume_raygeneration_fbo_, math::vec4f(0, 0, 0, 0.f));
            ctx.render_context->clear_depth_stencil_buffer(volume_raygeneration_fbo_);

            for(auto const& node : scene.nodes[std::type_index(typeid(node::VolumeNode))])
            {
                auto volume_node = reinterpret_cast<gua::node::VolumeNode*>(node);
                auto volume = std::static_pointer_cast<gua::Volume>(GeometryDatabase::instance()->lookup(volume_node->data.get_volume()));

                if(volume)
                {
                    ray_generation_shader_->set_uniform(ctx, math::mat4f(node->get_world_transform()), "gua_model_matrix");
                    ray_generation_shader_->use(ctx);
                    volume->draw_proxy(ctx);
                    ray_generation_shader_->unuse(ctx);
                }
            }
        }
        ctx.render_context->reset_framebuffer();

        scm::gl::context_all_guard cug(ctx.render_context);

        ctx.render_context->set_blend_state(blend_state_);

        // 2. render fullscreen quad for compositing and volume ray casting
        bool write_depth = false;
        target.bind(ctx, write_depth);
        pipe.bind_gbuffer_input(composite_shader_);
        composite_shader_->set_uniform(ctx, volume_raygeneration_color_buffer_->get_handle(ctx), "gua_ray_entry_in");

        for(auto const& node : scene.nodes[std::type_index(typeid(node::VolumeNode))])
        {
            auto volume_node = reinterpret_cast<gua::node::VolumeNode*>(node);
            auto volume = std::static_pointer_cast<gua::Volume>(GeometryDatabase::instance()->lookup(volume_node->data.get_volume()));

            if(volume)
            {
                composite_shader_->set_uniform(ctx, math::mat4f(node->get_world_transform()), "gua_model_matrix");

                volume->set_transfer_function(volume_node->data.alpha_transfer(), volume_node->data.color_transfer());
                volume->set_uniforms(ctx, composite_shader_.get());

                composite_shader_->use(ctx);
                pipe.draw_quad();
                composite_shader_->unuse(ctx);
            }
        }

        target.unbind(ctx);
        ctx.render_context->reset_state_objects();
    }
}

////////////////////////////////////////////////////////////////////////////////

void VolumeRenderer::init_resources(Pipeline& pipe)
{
    auto const& ctx(pipe.get_context());
    auto& target = *pipe.current_viewstate().target;

    scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE, scm::gl::WRAP_CLAMP_TO_EDGE);

    volume_raygeneration_color_buffer_ = std::make_shared<Texture2D>(target.get_width(), target.get_height(), scm::gl::FORMAT_RGB_32F, 1, state);

    volume_raygeneration_depth_buffer_ = std::make_shared<Texture2D>(target.get_width(), target.get_height(), scm::gl::FORMAT_D24, 1, state);

    volume_raygeneration_fbo_ = ctx.render_device->create_frame_buffer();
    volume_raygeneration_fbo_->attach_color_buffer(0, volume_raygeneration_color_buffer_->get_buffer(ctx), 0, 0);
    volume_raygeneration_fbo_->attach_depth_stencil_buffer(volume_raygeneration_depth_buffer_->get_buffer(ctx), 0, 0);

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    std::string vertex_shader = program_factory_.read_shader_file("resources/shaders/common/fullscreen_quad.vert");
    std::string fragment_shader = program_factory_.read_shader_file("resources/shaders/volume_compose.frag");
#else
    std::string vertex_shader(Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_vert));
    std::string fragment_shader(Resources::lookup_shader(Resources::shaders_volume_compose_frag));
#endif

    composite_shader_ = std::make_shared<ShaderProgram>();
    composite_shader_->create_from_sources(vertex_shader, fragment_shader);

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    std::string ray_generation_vertex_shader = program_factory_.read_shader_file("resources/shaders/volume_ray_generation.vert");
    std::string ray_generation_fragment_shader = program_factory_.read_shader_file("resources/shaders/volume_ray_generation.frag");
#else
    std::string ray_generation_vertex_shader(Resources::lookup_shader(Resources::shaders_volume_ray_generation_vert));
    std::string ray_generation_fragment_shader(Resources::lookup_shader(Resources::shaders_volume_ray_generation_frag));
#endif

    ray_generation_shader_ = std::make_shared<ShaderProgram>();
    ray_generation_shader_->create_from_sources(ray_generation_vertex_shader, ray_generation_fragment_shader);

    depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER, true, 1, 0, scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL));

    blend_state_ = ctx.render_device->create_blend_state(true, scm::gl::FUNC_SRC_ALPHA, scm::gl::FUNC_ONE_MINUS_SRC_ALPHA, scm::gl::FUNC_SRC_ALPHA, scm::gl::FUNC_ONE_MINUS_SRC_ALPHA);
}

} // namespace gua
