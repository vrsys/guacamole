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

#include <gua/renderer/NormalizationSubRenderer.hpp>
#include <lamure/ren/controller.h>

namespace gua
{
NormalizationSubRenderer::NormalizationSubRenderer() : PLodSubRenderer(), fullscreen_quad_(nullptr) { _load_shaders(); }

void NormalizationSubRenderer::create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims, gua::plod_shared_resources& shared_resources)
{
    nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
}

void NormalizationSubRenderer::render_sub_pass(Pipeline& pipe,
                                               PipelinePassDescription const& desc,
                                               gua::plod_shared_resources& shared_resources,
                                               std::vector<node::Node*>& sorted_models,
                                               std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t>>& nodes_in_frustum_per_model,
                                               lamure::context_t context_id,
                                               lamure::view_t lamure_view_id,
                                               bool render_multiview)
{
    RenderContext const& ctx(pipe.get_context());
    auto& target = *pipe.current_viewstate().target;

    scm::gl::context_all_guard context_guard(ctx.render_context);

    _check_for_shader_program();

    assert(shader_program_);

    if(!fullscreen_quad_)
    {
        fullscreen_quad_.reset(new scm::gl::quad_geometry(ctx.render_device, scm::math::vec2(-1.0f, -1.0f), scm::math::vec2(1.0f, 1.0f)));
    }
    bool write_depth = true;
    target.bind(ctx, write_depth);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    std::string const gpu_query_name_normalization_pass = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLodRenderer::NormalizationPass";
    pipe.begin_gpu_query(ctx, gpu_query_name_normalization_pass);
#endif

    shader_program_->use(ctx);
    {
        _upload_normalization_pass_uniforms(ctx, shared_resources);

        ctx.render_context->apply();

        fullscreen_quad_->draw(ctx.render_context);
    }
    shader_program_->unuse(ctx);

    target.unbind(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    pipe.end_gpu_query(ctx, gpu_query_name_normalization_pass);
#endif
}

void NormalizationSubRenderer::_load_shaders()
{
    // create stages only with one thread!
    if(!shaders_loaded_)
    {
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
        ResourceFactory factory;

        shader_stages_.clear();
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/plod/two_pass_splatting/p03_normalization.vert")));
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/plod/two_pass_splatting/p03_normalization.frag")));

        shaders_loaded_ = true;
    }
}

void NormalizationSubRenderer::_upload_normalization_pass_uniforms(RenderContext const& ctx, gua::plod_shared_resources& shared_resources)
{
    ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_COLOR_RESULT], nearest_sampler_state_, 0);
    shader_program_->apply_uniform(ctx, "p02_color_texture", 0);

    ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_NORMAL_RESULT], nearest_sampler_state_, 1);
    shader_program_->apply_uniform(ctx, "p02_normal_texture", 1);

    ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_PBR_RESULT], nearest_sampler_state_, 2);
    shader_program_->apply_uniform(ctx, "p02_pbr_texture", 2);

    ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_WEIGHT_AND_DEPTH_RESULT], nearest_sampler_state_, 3);
    shader_program_->apply_uniform(ctx, "p02_weight_and_depth_texture", 3);
}

} // namespace gua