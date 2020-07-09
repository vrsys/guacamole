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

#include <gua/renderer/AccumSubRenderer.hpp>
#include <lamure/ren/controller.h>

#include <gua/databases/TimeSeriesDataSetDatabase.hpp>

namespace gua
{
AccumSubRenderer::AccumSubRenderer() : PLodSubRenderer() { _load_shaders(); }

void AccumSubRenderer::create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims, gua::plod_shared_resources& shared_resources)
{
    // initialize FBO lazy during runtime
    custom_FBO_ptr_.reset();

    // attachments
    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_COLOR_RESULT] = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RGB_16F, 1, 1, 1);

    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_NORMAL_RESULT] = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RGB_16F, 1, 1, 1);

    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_PBR_RESULT] = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RGB_16F, 1, 1, 1);

    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_WEIGHT_AND_DEPTH_RESULT] = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RG_32F, 1, 1, 1);

    depth_test_without_writing_depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(true, false, scm::gl::COMPARISON_LESS_EQUAL);

    color_accumulation_state_ = ctx.render_device->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::EQ_FUNC_ADD, scm::gl::EQ_FUNC_ADD);

    no_backface_culling_rasterizer_state_ =
        ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false, scm::gl::point_raster_state(false));

    _register_shared_resources(shared_resources);
}

float dummy_generator() {
    static float gen = 0.0f;
    return gen += 1.0f;
}


void AccumSubRenderer::render_sub_pass(Pipeline& pipe,
                                       PipelinePassDescription const& desc,
                                       gua::plod_shared_resources& shared_resources,
                                       std::vector<node::Node*>& sorted_models,
                                       std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t>>& nodes_in_frustum_per_model,
                                       lamure::context_t context_id,
                                       lamure::view_t lamure_view_id,
                                       bool render_multiview)
{

    auto const& camera = pipe.current_viewstate().camera;
    RenderContext& ctx(pipe.get_context());
    lamure::ren::controller* controller = lamure::ren::controller::get_instance();

    scm::gl::context_all_guard context_guard(ctx.render_context);

    if(!custom_FBO_ptr_)
    {
        custom_FBO_ptr_ = ctx.render_device->create_frame_buffer();
        custom_FBO_ptr_->clear_attachments();
        custom_FBO_ptr_->attach_color_buffer(0, shared_resources.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_COLOR_RESULT]);
        custom_FBO_ptr_->attach_color_buffer(1, shared_resources.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_NORMAL_RESULT]);
        custom_FBO_ptr_->attach_color_buffer(2, shared_resources.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_PBR_RESULT]);
        custom_FBO_ptr_->attach_color_buffer(3, shared_resources.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_WEIGHT_AND_DEPTH_RESULT]);
    }

    ctx.render_context->clear_color_buffer(custom_FBO_ptr_, 0, scm::math::vec3f(0.0f, 0.0f, 0.0f));

    ctx.render_context->clear_color_buffer(custom_FBO_ptr_, 1, scm::math::vec3f(0.0f, 0.0f, 0.0f));

    ctx.render_context->clear_color_buffer(custom_FBO_ptr_, 2, scm::math::vec3f(0.0f, 0.0f, 0.0f));

    ctx.render_context->clear_color_buffer(custom_FBO_ptr_, 3, scm::math::vec2f(0.0f, 0.0f));

    MaterialShader* current_material(nullptr);
    std::shared_ptr<ShaderProgram> current_material_program = nullptr;

    {
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        std::string const gpu_query_name_accum_pass = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / AccumSubRenderer::AccumulationPass";
        pipe.begin_gpu_query(ctx, gpu_query_name_accum_pass);
#endif

        // set accumulation pass states
        ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);
        ctx.render_context->set_depth_stencil_state(depth_test_without_writing_depth_stencil_state_);
        ctx.render_context->set_blend_state(color_accumulation_state_);
        ctx.render_context->set_frame_buffer(custom_FBO_ptr_);
        custom_FBO_ptr_->attach_depth_stencil_buffer(shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH]);

        int view_id(camera.config.get_view_id());

        uint32_t num_different_models = sorted_models.size();


        bool needs_resource_rebinding = num_different_models != 1;
        bool program_changed = false;
        // loop through all models and render accumulation pass
        for(auto const& object : sorted_models)
        {
            auto plod_node(reinterpret_cast<node::PLodNode*>(object));
            lamure::model_t model_id = controller->deduce_model_id(plod_node->get_geometry_description());

            current_material = plod_node->get_material()->get_shader();

            auto new_plod_node_material = _get_material_dependent_shader_program(current_material, current_material_program, program_changed);

            if(current_material_program != new_plod_node_material) {

                current_material_program = new_plod_node_material;
                current_material_program->apply_uniform(ctx, "p01_linear_depth_texture", 0);

                //current_material_program->set_uniform(ctx, int(20), "dummy_value_ssbo_layout");

                //ctx.render_context->bind_storage_buffer( ctx.shader_storage_buffer_objects.begin()->second, 20, 0, 1000);
                //ctx.render_context->apply_storage_buffer_bindings();
            }

            ctx.render_context->apply();

            auto plod_resource = plod_node->get_geometry();


            // retrieve frustum culling results
            std::unordered_set<lamure::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[plod_node];

            if(plod_resource && current_material_program)
            {
                if(program_changed)
                {
                    //current_material_program->unuse(ctx);
                    current_material_program->use(ctx);
                }

                _upload_model_dependent_uniforms(current_material_program, ctx, plod_node, pipe);

                plod_node->get_material()->apply_uniforms(ctx, current_material_program.get(), view_id);
                plod_node->bind_time_series_data_to(ctx, current_material_program);

                ctx.render_context->apply();

                plod_resource->draw(ctx,
                                    context_id,
                                    lamure_view_id,
                                    model_id,
                                    controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, ctx.render_device),
                                    nodes_in_frustum,
                                    scm::gl::primitive_topology::PRIMITIVE_POINT_LIST );

                program_changed = false;
            }
            else
            {
                Logger::LOG_WARNING << "AccumSubRenderer::render(): Cannot find ressources for node: " << plod_node->get_name() << std::endl;
            }
        }

        //current_material_program->unuse(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        pipe.end_gpu_query(ctx, gpu_query_name_accum_pass);
#endif
    }
}

void AccumSubRenderer::_load_shaders()
{
    // create stages only with one thread!
    if(!shaders_loaded_)
    {
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
        ResourceFactory factory;
        shader_stages_.clear();
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/plod/two_pass_splatting/p02_accum.vert")));
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/plod/two_pass_splatting/p02_accum.geom")));
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/plod/two_pass_splatting/p02_accum.frag")));
        shaders_loaded_ = true;
    }
}

void AccumSubRenderer::_upload_model_dependent_uniforms(std::shared_ptr<ShaderProgram> current_material_shader, RenderContext const& ctx, node::PLodNode* plod_node, gua::Pipeline& pipe)
{
    auto const& frustum = pipe.current_viewstate().frustum;

    auto const& scm_model_matrix = plod_node->get_cached_world_transform();
    auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
    auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
    auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));
    auto scm_inv_trans_model_view_matrix = scm::math::transpose(scm::math::inverse(scm_model_view_matrix));

    current_material_shader->apply_uniform(ctx, "gua_model_matrix", math::mat4f(scm_model_matrix));
    current_material_shader->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(scm_model_view_matrix));
    current_material_shader->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(scm_normal_matrix));
    current_material_shader->apply_uniform(ctx, "gua_model_view_projection_matrix", math::mat4f(scm_model_view_projection_matrix));
    current_material_shader->apply_uniform(ctx, "inverse_transpose_model_view_matrix", math::mat4f(scm_inv_trans_model_view_matrix));

    current_material_shader->apply_uniform(ctx, "radius_scaling", plod_node->get_radius_scale());
    current_material_shader->apply_uniform(ctx, "max_surfel_radius", plod_node->get_max_surfel_radius());
    current_material_shader->apply_uniform(ctx, "enable_backface_culling", plod_node->get_enable_backface_culling_by_normal());

    current_material_shader->apply_uniform(ctx, "has_provenance_attributes", plod_node->get_has_provenance_attributes());
}

} // namespace gua