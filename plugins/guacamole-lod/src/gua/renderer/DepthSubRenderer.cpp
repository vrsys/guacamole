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

#include <gua/renderer/DepthSubRenderer.hpp>
#include <lamure/ren/controller.h>
#include <gua/databases/TimeSeriesDataSetDatabase.hpp>

namespace gua
{
DepthSubRenderer::DepthSubRenderer() : PLodSubRenderer() { _load_shaders(); }

void DepthSubRenderer::create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims, gua::plod_shared_resources& shared_resources)
{
    // initialize FBO lazy during runtime
    custom_FBO_ptr_.reset();

    // state objects
    no_backface_culling_rasterizer_state_ =
        ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false, scm::gl::point_raster_state(false));
}

void DepthSubRenderer::render_sub_pass(Pipeline& pipe,
                                       PipelinePassDescription const& desc,
                                       gua::plod_shared_resources& shared_resources,
                                       std::vector<node::Node*>& sorted_models,
                                       std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t>>& nodes_in_frustum_per_model,
                                       lamure::context_t context_id,
                                       lamure::view_t lamure_view_id,
                                       bool render_multiview)
{
    RenderContext& ctx(pipe.get_context());
    scm::gl::context_all_guard context_guard(ctx.render_context);

    if(!custom_FBO_ptr_)
    {
        custom_FBO_ptr_ = ctx.render_device->create_frame_buffer();
        custom_FBO_ptr_->clear_attachments();
        custom_FBO_ptr_->attach_depth_stencil_buffer(shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH]);
    }

    _check_for_shader_program();

    assert(shader_program_);

    ctx.render_context->clear_color_buffer(custom_FBO_ptr_, 0, scm::math::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);
    custom_FBO_ptr_->attach_depth_stencil_buffer(shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH]);
    ctx.render_context->set_frame_buffer(custom_FBO_ptr_);

    shader_program_->use(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    std::string const gpu_query_name_depth_pass = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / DepthSubRenderer::DepthPass";
    pipe.begin_gpu_query(ctx, gpu_query_name_depth_pass);
#endif

    // loop through all models and render depth pass
    for(auto const& object : sorted_models)
    {
        auto plod_node(reinterpret_cast<node::PLodNode*>(object));

        _upload_model_dependent_uniforms(ctx, plod_node, pipe);

        ctx.render_context->apply();

        lamure::ren::controller* controller = lamure::ren::controller::get_instance();

        lamure::model_t model_id = controller->deduce_model_id(plod_node->get_geometry_description());

        std::unordered_set<lamure::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[plod_node];

        auto const& plod_resource = plod_node->get_geometry();




        if(plod_resource && shader_program_)
        {
            plod_node->bind_time_series_data_to(ctx, shader_program_);

            plod_resource->draw(ctx,
                                context_id,
                                lamure_view_id,
                                model_id,
                                controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, ctx.render_device),
                                nodes_in_frustum,
                                scm::gl::primitive_topology::PRIMITIVE_POINT_LIST);
        }
        else
        {
            Logger::LOG_WARNING << "DepthSubRenderer::render(): Cannot find ressources for node: " << plod_node->get_name() << std::endl;
        }
    }

    //shader_program_->unuse(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    pipe.end_gpu_query(ctx, gpu_query_name_depth_pass);
#endif
}

void DepthSubRenderer::_load_shaders()
{
    // create stages only with one thread!
    if(!shaders_loaded_)
    {
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
        ResourceFactory factory;
        shader_stages_.clear();
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/plod/two_pass_splatting/p01_depth.vert")));
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/plod/two_pass_splatting/p01_depth.geom")));
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/plod/two_pass_splatting/p01_depth.frag")));

        shaders_loaded_ = true;
    }
}

void DepthSubRenderer::_upload_model_dependent_uniforms(RenderContext const& ctx, node::PLodNode* plod_node, gua::Pipeline& pipe)
{
    auto const& frustum = pipe.current_viewstate().frustum;

    auto const& scm_model_matrix = plod_node->get_cached_world_transform();
    auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
    auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
    auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));
    auto scm_inv_trans_model_view_matrix = scm::math::transpose(scm::math::inverse(scm_model_view_matrix));

    shader_program_->apply_uniform(ctx, "gua_model_matrix", math::mat4f(scm_model_matrix));
    shader_program_->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(scm_model_view_matrix));
    shader_program_->apply_uniform(ctx, "gua_model_view_projection_matrix", math::mat4f(scm_model_view_projection_matrix));
    shader_program_->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(scm_normal_matrix));
    shader_program_->apply_uniform(ctx, "inverse_transpose_model_view_matrix", math::mat4f(scm_inv_trans_model_view_matrix));

    shader_program_->apply_uniform(ctx, "radius_scaling", plod_node->get_radius_scale());
    shader_program_->apply_uniform(ctx, "max_surfel_radius", plod_node->get_max_surfel_radius());
    shader_program_->apply_uniform(ctx, "enable_backface_culling", plod_node->get_enable_backface_culling_by_normal());
    shader_program_->apply_uniform(ctx, "has_provenance_attributes", plod_node->get_has_provenance_attributes());
}

} // namespace gua