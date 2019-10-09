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

#include <gua/renderer/LodResource.hpp>

#include <gua/renderer/LogToLinSubRenderer.hpp>
#include <lamure/ren/controller.h>

namespace gua
{
LogToLinSubRenderer::LogToLinSubRenderer() : PLodSubRenderer(), fullscreen_quad_(nullptr), nearest_sampler_state_(nullptr), no_depth_test_with_writing_depth_stencil_state_(nullptr)
{
    _load_shaders();
}

void LogToLinSubRenderer::create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims, gua::plod_shared_resources& shared_resources)
{
    // attachments
    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH] = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_D32F, 1, 1, 1);

    _register_shared_resources(shared_resources);
}

void LogToLinSubRenderer::render_sub_pass(Pipeline& pipe,
                                          PipelinePassDescription const& desc,
                                          gua::plod_shared_resources& shared_resources,
                                          std::vector<node::Node*>& sorted_models,
                                          std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t>>& nodes_in_frustum_per_model,
                                          lamure::context_t context_id,
                                          lamure::view_t lamure_view_id)
{
    RenderContext const& ctx(pipe.get_context());
    auto& target = *pipe.current_viewstate().target;
    auto const& camera = pipe.current_viewstate().camera;

    scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();

    scm::gl::context_all_guard context_guard(ctx.render_context);
    auto& gua_depth_buffer = target.get_depth_buffer();

    _check_for_shader_program();

    assert(shader_program_);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    std::string const gpu_query_name_depth_conversion = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLodRenderer::LogToLinConversionPass";
    pipe.begin_gpu_query(ctx, gpu_query_name_depth_conversion);
#endif

    if(!custom_FBO_ptr_)
    {
        custom_FBO_ptr_ = ctx.render_device->create_frame_buffer();
    }

    if(!fullscreen_quad_)
    {
        fullscreen_quad_.reset(new scm::gl::quad_geometry(ctx.render_device, scm::math::vec2(-1.0f, -1.0f), scm::math::vec2(1.0f, 1.0f)));
    }

    shader_program_->use(ctx);

    if(!no_depth_test_with_writing_depth_stencil_state_)
    {
        no_depth_test_with_writing_depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_ALWAYS);
    }

    if(!nearest_sampler_state_)
    {
        nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
    }

    ctx.render_context->set_depth_stencil_state(no_depth_test_with_writing_depth_stencil_state_);

    custom_FBO_ptr_->attach_depth_stencil_buffer(shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH]);

    ctx.render_context->set_frame_buffer(custom_FBO_ptr_);

    ctx.render_context->bind_texture(gua_depth_buffer, nearest_sampler_state_, 0);

    shader_program_->apply_uniform(ctx, "gua_log_depth_buffer", 0);

    float width = render_target_dims[0];
    float height = render_target_dims[1];
    shader_program_->apply_uniform(ctx, "win_width", (width));

    shader_program_->apply_uniform(ctx, "win_height", (height));

    ctx.render_context->apply();

    fullscreen_quad_->draw(ctx.render_context);

    shader_program_->unuse(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    pipe.end_gpu_query(ctx, gpu_query_name_depth_conversion);
#endif
}

void LogToLinSubRenderer::_load_shaders()
{
    // create stages only with one thread!
    if(!shaders_loaded_)
    {
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
        ResourceFactory factory;

        shader_stages_.clear();
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/plod/p00_log_to_lin_conversion.vert")));
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/plod/p00_log_to_lin_conversion.frag")));

        shaders_loaded_ = true;
    }
}

} // namespace gua
