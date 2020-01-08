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
#include <gua/renderer/LowQualitySplattingSubRenderer.hpp>

#include <gua/renderer/LodResource.hpp>

#include <lamure/ren/controller.h>

#include <gua/databases/TimeSeriesDataSetDatabase.hpp>

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
LowQualitySplattingSubRenderer::LowQualitySplattingSubRenderer() : PLodSubRenderer() { _load_shaders(); }

void LowQualitySplattingSubRenderer::create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims, gua::plod_shared_resources& shared_resources)
{
    // state objects
    no_backface_culling_rasterizer_state_ =
        ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false, scm::gl::point_raster_state(false));
}

void LowQualitySplattingSubRenderer::render_sub_pass(Pipeline& pipe,
                                                     PipelinePassDescription const& desc,
                                                     gua::plod_shared_resources& shared_resources,
                                                     std::vector<node::Node*>& sorted_models,
                                                     std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t>>& nodes_in_frustum_per_model,
                                                     lamure::context_t context_id,
                                                     lamure::view_t lamure_view_id)
{
    auto const& camera = pipe.current_viewstate().camera;
    RenderContext& ctx(pipe.get_context());
    auto& target = *pipe.current_viewstate().target;

    scm::gl::context_all_guard context_guard(ctx.render_context);

    ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);

    bool write_depth = true;
    target.bind(ctx, write_depth);
    target.set_viewport(ctx);

    MaterialShader* current_material(nullptr);
    std::shared_ptr<ShaderProgram> current_material_program;
    bool program_changed = false;

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    std::string const gpu_query_name_depth_pass = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLodRenderer::DepthPass";
    pipe.begin_gpu_query(ctx, gpu_query_name_depth_pass);
#endif

    int view_id(camera.config.get_view_id());

    // loop through all models and render depth pass
    for(auto const& object : sorted_models)
    {
        auto plod_node(reinterpret_cast<node::PLodNode*>(object));

        current_material = plod_node->get_material()->get_shader();

        current_material_program = _get_material_dependent_shader_program(current_material, current_material_program, program_changed);

        lamure::ren::controller* controller = lamure::ren::controller::get_instance();
        lamure::model_t model_id = controller->deduce_model_id(plod_node->get_geometry_description());
        std::unordered_set<lamure::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[plod_node];

        auto const& plod_resource = plod_node->get_geometry();

        if(plod_resource && current_material_program)
        {
            if(program_changed)
            {
                current_material_program->unuse(ctx);
                current_material_program->use(ctx);
                current_material_program->set_uniform(ctx, ::get_handle(target.get_depth_buffer()), "gua_gbuffer_depth");
            }

            plod_node->get_material()->apply_uniforms(ctx, current_material_program.get(), view_id);
            _upload_model_dependent_uniforms(current_material_program, ctx, plod_node, pipe);



            auto time_series_data_descriptions = plod_node->get_time_series_data_descriptions();

            //std::cout << "Time series data description size: " << time_series_data_descriptions.size() << std::endl;

            for(auto const& data_description : time_series_data_descriptions) {
                auto looked_up_time_series_data_item = TimeSeriesDataSetDatabase::instance()->lookup(data_description);

                if(looked_up_time_series_data_item) {
                    //std::cout << "FOUND DATA ITEM" << std::endl;
                    //std::cout << looked_up_time_series_data_item->data.size() << std::endl;


                    looked_up_time_series_data_item->upload_time_range_to(ctx);
                }
            }


            
            if( !plod_node->get_time_series_data_descriptions().empty() ) {

                auto active_time_series_index = plod_node->get_active_time_series_index();

                auto const& active_time_series_description = time_series_data_descriptions[active_time_series_index];

                //for(auto const& data_description : time_series_data_descriptions) {
                //auto looked_up_time_series_data_item = TimeSeriesDataSetDatabase::instance()->lookup(active_time_series_description);


                //for(auto const& data_description : time_series_data_descriptions) {
                auto looked_up_time_series_data_item = TimeSeriesDataSetDatabase::instance()->lookup(active_time_series_description);
                //auto current_ssbo_ptr_it = ctx.shader_storage_buffer_objects.find(looked_up_time_series_data_item->uuid);


                int32_t attribute_to_visualize_index = plod_node->get_attribute_to_visualize_index();
                looked_up_time_series_data_item->bind_to(ctx, 20, current_material_program, attribute_to_visualize_index);

                //int32_t current_timestep_offset = int(ctx.framecount % 100);

                float current_timecursor_position = plod_node->get_time_cursor_position();

                current_timecursor_position = looked_up_time_series_data_item->calculate_active_cursor_position(current_timecursor_position);


                /*
                if( (looked_up_time_series_data_item->num_timesteps != 1) && (looked_up_time_series_data_item->sequence_length != 0.0f) ) {
                    if(current_timecursor_position >  looked_up_time_series_data_item->sequence_length) {
                        current_timecursor_position = std::fmod(current_timecursor_position, looked_up_time_series_data_item->sequence_length);
                    }

                    current_timecursor_position /= (looked_up_time_series_data_item->sequence_length/looked_up_time_series_data_item->num_timesteps );
                } else {
                    current_timecursor_position = 0.0f;
                }

                */

                std::cout << "GOING TO UPLOAD TIMECURSOR POSITION: " << current_timecursor_position << std::endl;

                current_material_program->set_uniform(ctx, current_timecursor_position, "current_timestep");

                current_material_program->set_uniform(ctx, plod_node->get_enable_time_series_coloring(), "enable_time_series_coloring");
                current_material_program->set_uniform(ctx, plod_node->get_enable_time_series_deformation(), "enable_time_series_deformation");
                current_material_program->set_uniform(ctx, plod_node->get_time_series_deform_factor(), "deform_factor");    


                //}

            }


            ctx.render_context->apply();

            plod_resource->draw(ctx,
                                context_id,
                                lamure_view_id,
                                model_id,
                                controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, ctx.render_device),
                                nodes_in_frustum,
                                scm::gl::primitive_topology::PRIMITIVE_POINT_LIST);

            program_changed = false;
        }
        else
        {
            Logger::LOG_WARNING << "PLodRenderer::render(): Cannot find ressources for node: " << plod_node->get_name() << std::endl;
        }
    }
    target.unbind(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    pipe.end_gpu_query(ctx, gpu_query_name_depth_pass);
#endif
}

void LowQualitySplattingSubRenderer::_load_shaders()
{
    // create stages only with one thread!
    if(!shaders_loaded_)
    {
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
        ResourceFactory factory;
        shader_stages_.clear();
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/plod/one_pass_splatting/one_pass_low_quality.vert")));
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/plod/one_pass_splatting/one_pass_low_quality.geom")));
        shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/plod/one_pass_splatting/one_pass_low_quality.frag")));
        shaders_loaded_ = true;
    }
}

void LowQualitySplattingSubRenderer::_upload_model_dependent_uniforms(std::shared_ptr<ShaderProgram> const& current_material_shader,
                                                                      RenderContext const& ctx,
                                                                      node::PLodNode* plod_node,
                                                                      gua::Pipeline& pipe)
{
    auto const& frustum = pipe.current_viewstate().frustum;

    auto const& scm_model_matrix = plod_node->get_cached_world_transform();
    auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
    auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
    auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));
    auto scm_inv_trans_model_view_matrix = scm::math::transpose(scm::math::inverse(scm_model_view_matrix));

    current_material_shader->apply_uniform(ctx, "gua_model_matrix", math::mat4f(scm_model_matrix));
    current_material_shader->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(scm_model_view_matrix));
    current_material_shader->apply_uniform(ctx, "gua_model_view_projection_matrix", math::mat4f(scm_model_view_projection_matrix));
    current_material_shader->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(scm_normal_matrix));
    current_material_shader->apply_uniform(ctx, "inverse_transpose_model_view_matrix", math::mat4f(scm_inv_trans_model_view_matrix));

    current_material_shader->apply_uniform(ctx, "radius_scaling", plod_node->get_radius_scale());
    current_material_shader->apply_uniform(ctx, "max_surfel_radius", plod_node->get_max_surfel_radius());
    current_material_shader->apply_uniform(ctx, "enable_backface_culling", plod_node->get_enable_backface_culling_by_normal());
}

} // namespace gua
