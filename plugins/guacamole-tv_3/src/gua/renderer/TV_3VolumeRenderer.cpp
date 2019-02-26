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
#include <gua/renderer/TV_3VolumeRenderer.hpp>
#include <gua/renderer/TV_3VolumePass.hpp>

// guacamole headers
#include <gua/renderer/TV_3Resource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ResourceFactory.hpp>

#include <gua/node/TV_3Node.hpp>
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/renderer/View.hpp>

#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>

#include <gua/renderer/Window.hpp>

#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/data_types.h>
#include <scm/gl_core/texture_objects/texture_image.h>

#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/render_device.h>

// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>
#include <boost/assign/list_of.hpp>

namespace gua
{
//////////////////////////////////////////////////////////////////////////////
TV_3VolumeRenderer::TV_3VolumeRenderer(gua::RenderContext const& ctx, gua::SubstitutionMap const& smap) : TV_3Renderer(ctx, smap) {}

///////////////////////////////////////////////////////////////////////////////
void TV_3VolumeRenderer::_create_fbo_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims)
{
    // initialize FBO lazy during runtime
    volume_raycasting_fbo_.reset();

    // attachments
    volume_raycasting_back_buffer_color_result_ = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RGBA_8, 1, 1, 1);

    volume_raycasting_front_buffer_color_result_ = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RGBA_8, 1, 1, 1);

    volume_raycasting_back_buffer_depth_result_ = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_D32F, 1, 1, 1);

    volume_raycasting_front_buffer_depth_result_ = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_D32F, 1, 1, 1);

    /*
        volume_compositing_blend_state_ = ctx.render_device->create_blend_state(true,
                                                                          scm::gl::FUNC_SRC_ALPHA,
                                                                          scm::gl::FUNC_ONE_MINUS_SRC_ALPHA,
                                                                          scm::gl::FUNC_SRC_ALPHA,
                                                                          scm::gl::FUNC_ONE_MINUS_SRC_ALPHA,
                                                                          scm::gl::EQ_FUNC_ADD,
                                                                          scm::gl::EQ_FUNC_ADD);
    */
    no_blending_blend_state_ = ctx.render_device->create_blend_state(false);
}

// called once per frame. This is the only instance in which the front buffers should be cleared
void TV_3VolumeRenderer::_clear_fbo_attachments(gua::RenderContext const& ctx)
{
    if(!volume_raycasting_fbo_)
    {
        volume_raycasting_fbo_ = ctx.render_device->create_frame_buffer();
        volume_raycasting_fbo_->clear_attachments();
        volume_raycasting_fbo_->attach_color_buffer(0, volume_raycasting_back_buffer_color_result_);

        volume_raycasting_fbo_->attach_depth_stencil_buffer(volume_raycasting_back_buffer_depth_result_);

        ctx.render_context->set_frame_buffer(volume_raycasting_fbo_);

        volume_compositing_fbo_ = ctx.render_device->create_frame_buffer();
    }

    ////////////////////////////
    // clear front buffers START
    volume_compositing_fbo_->clear_attachments();
    volume_compositing_fbo_->attach_color_buffer(0, volume_raycasting_front_buffer_color_result_);
    // volume_compositing_fbo_
    //  ->attach_depth_stencil_buffer(volume_raycasting_front_buffer_depth_result_);

    ctx.render_context->clear_color_buffer(volume_compositing_fbo_, 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));

    // ctx.render_context
    //  ->clear_depth_stencil_buffer(volume_compositing_fbo_);
    // clear front buffers END
    ////////////////////////////

    ////////////////////////////
    // clear back buffers START
    volume_raycasting_fbo_->clear_attachments();
    volume_raycasting_fbo_->attach_color_buffer(0, volume_raycasting_back_buffer_color_result_);
    // volume_raycasting_fbo_
    //  ->attach_depth_stencil_buffer(volume_raycasting_back_buffer_depth_result_);

    ctx.render_context->clear_color_buffer(volume_raycasting_fbo_, 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));

    // ctx.render_context
    //  ->clear_depth_stencil_buffer(volume_raycasting_fbo_);
    // clear back buffers END
    ////////////////////////////

    // keep the back buffers attached

    ctx.render_context->set_frame_buffer(volume_raycasting_fbo_);
}

void TV_3VolumeRenderer::_load_shaders()
{
    TV_3Renderer::_load_shaders();

#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
    ResourceFactory factory;
    forward_cube_shader_stages_.clear();
    forward_cube_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/tv_3/ray_casting.vert")));
    forward_cube_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/tv_3/ray_casting.frag")));

    {
        auto new_program = std::make_shared<ShaderProgram>();
        new_program->set_shaders(forward_cube_shader_stages_);
        forward_cube_shader_program_ = new_program;
    }

    compositing_shader_stages_.clear();
    compositing_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/tv_3/fullscreen_blit.vert")));
    compositing_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/tv_3/fullscreen_blit.frag")));

    {
        auto new_program = std::make_shared<ShaderProgram>();
        new_program->set_shaders(compositing_shader_stages_);
        compositing_shader_program_ = new_program;
    }

    volume_compositing_shader_stages_.clear();
    volume_compositing_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/tv_3/fullscreen_blit.vert")));
    volume_compositing_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/tv_3/volume_fullscreen_compositing.frag")));

    {
        auto new_program = std::make_shared<ShaderProgram>();
        new_program->set_shaders(volume_compositing_shader_stages_);
        volume_compositing_shader_program_ = new_program;
    }

    shaders_loaded_ = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void TV_3VolumeRenderer::_raycasting_pass(gua::Pipeline& pipe, std::vector<gua::node::Node*> const& sorted_nodes, PipelinePassDescription const& desc)
{
    RenderContext const& ctx(pipe.get_context());
    auto& scene = *pipe.current_viewstate().scene;

    auto view_matrix = scene.rendering_frustum.get_view();
    auto projection_matrix = scene.rendering_frustum.get_projection();

    auto eye_position = math::vec4(scene.rendering_frustum.get_camera_position(), 1.0);

    MaterialShader* current_material(nullptr);
    node::TV_3Node::RenderMode current_render_mode(node::TV_3Node::RenderMode::SUR_PBR);
    std::shared_ptr<ShaderProgram> current_material_program;
    bool program_changed = false;

    auto const& camera = pipe.current_viewstate().camera;
    int view_id(camera.config.get_view_id());

    std::vector<gua::node::TV_3Node*> front_to_back_sorted_volume_nodes;

    for(auto const& object : sorted_nodes)
    {
        auto tv_3_volume_node(reinterpret_cast<node::TV_3Node*>(object));
        if(node::TV_3Node::RenderMode::SUR_PBR == tv_3_volume_node->get_render_mode())
        {
            continue;
        }

        front_to_back_sorted_volume_nodes.push_back(const_cast<node::TV_3Node*>(reinterpret_cast<node::TV_3Node* const>(object)));
    }

    std::sort(front_to_back_sorted_volume_nodes.begin(), front_to_back_sorted_volume_nodes.end(), [&view_matrix](node::TV_3Node* a, node::TV_3Node* b) {
        auto model_matrix_a = a->get_cached_world_transform();
        auto model_view_matrix_a = view_matrix * model_matrix_a;
        auto const mv_bb_center_a = model_view_matrix_a * a->get_bounding_box().center();

        float three_d_squared_distance_bb_center_a = mv_bb_center_a[0] * mv_bb_center_a[0] + mv_bb_center_a[1] * mv_bb_center_a[1] + mv_bb_center_a[2] * mv_bb_center_a[2];

        auto model_matrix_b = b->get_cached_world_transform();
        auto model_view_matrix_b = view_matrix * model_matrix_b;
        auto const mv_bb_center_b = model_view_matrix_b * b->get_bounding_box().center();

        float three_d_squared_distance_bb_center_b = mv_bb_center_b[0] * mv_bb_center_b[0] + mv_bb_center_b[1] * mv_bb_center_b[1] + mv_bb_center_b[2] * mv_bb_center_b[2];

        return three_d_squared_distance_bb_center_a > three_d_squared_distance_bb_center_b;
    });

    int node_counter = 0;
    for(auto const& tv_3_volume_node : front_to_back_sorted_volume_nodes)
    {
        // skip nodes which do not fit our pass description

        if(node::TV_3Node::RenderMode::SUR_PBR == tv_3_volume_node->get_render_mode())
        {
            continue;
        }

        // after the compositing for this pass is done, swap the buffers
        // std::swap(volume_raycasting_front_buffer_color_result_, volume_raycasting_back_buffer_color_result_);
        // std::swap(volume_raycasting_front_buffer_depth_result_, volume_raycasting_back_buffer_depth_result_);

        // and clear the write buffer (because we don't render fullscreen quads & therefore do not overwrite every pixel)

        volume_raycasting_fbo_->attach_color_buffer(0, volume_raycasting_back_buffer_color_result_);
        // volume_raycasting_fbo_
        // ->attach_depth_stencil_buffer(volume_raycasting_back_buffer_depth_result_);

        ctx.render_context->clear_color_buffer(volume_raycasting_fbo_, 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));

        // ctx.render_context
        // ->clear_depth_stencil_buffer(volume_raycasting_fbo_);

        ctx.render_context->set_frame_buffer(volume_raycasting_fbo_);

        if(current_material != tv_3_volume_node->get_material()->get_shader() || current_render_mode != tv_3_volume_node->get_render_mode())
        {
            current_material = tv_3_volume_node->get_material()->get_shader();

            auto compression_mode = tv_3_volume_node->get_compression_mode();
            auto spatial_filter_mode = tv_3_volume_node->get_spatial_filter_mode();
            auto temporal_filter_mode = tv_3_volume_node->get_temporal_filter_mode();
            auto node_render_mode = tv_3_volume_node->get_render_mode();
            current_material_program =
                _get_material_program(current_material, current_material_program, program_changed, compression_mode, spatial_filter_mode, temporal_filter_mode, node_render_mode);
        }

        auto model_matrix = tv_3_volume_node->get_cached_world_transform();
        auto mvp_matrix = projection_matrix * view_matrix * model_matrix;
        // forward_cube_shader_program_->apply_uniform(ctx, "gua_model_view_projection_matrix", math::mat4f(mvp_matrix));
        auto inv_model_mat = scm::math::inverse(model_matrix);

        math::vec4 model_space_eye_pos = inv_model_mat * eye_position;

        // forward_cube_shader_program_->apply_uniform(ctx, "gua_model_matrix", math::mat4f(tv_3_volume_node->get_world_transform()) ) ;

        auto normal_matrix = scm::math::transpose(scm::math::inverse(model_matrix));

        current_material_program->use(ctx);
        current_material_program->apply_uniform(ctx, "gua_model_matrix", math::mat4f(model_matrix));
        current_material_program->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(normal_matrix));
        current_material_program->apply_uniform(ctx, "gua_model_view_projection_matrix", math::mat4f(mvp_matrix));
        current_material_program->apply_uniform(ctx, "ms_eye_pos", math::vec4f(model_space_eye_pos / model_space_eye_pos[3]));
        current_material_program->apply_uniform(ctx, "volume_texture", 0);
        current_material_program->apply_uniform(ctx, "codebook_texture", 1);
        current_material_program->apply_uniform(ctx, "compositing_color_texture", 2);
        current_material_program->apply_uniform(ctx, "compositing_auxiliary_depth_buffer", 3);
        current_material_program->apply_uniform(ctx, "iso_value", tv_3_volume_node->get_iso_value());
        current_material_program->apply_uniform(ctx, "vol_idx", node_counter);

        // if(program_changed) {
        tv_3_volume_node->get_material()->apply_uniforms(ctx, current_material_program.get(), view_id);
        //}

        ctx.render_context->bind_vertex_array(box_vertex_array_);
        // ctx.render_context->bind_index_buffer(box_element_buffer_, scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

        ctx.render_context->set_rasterizer_state(frontface_culling_rasterizer_state_);
        // ctx.render_context->set_blend_state(volume_compositing_blend_state_);

        // auto& gl_api = ctx.render_context->opengl_api();
        // gl_api.glEnable(GL_BLEND);
        // gl_api.glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // ctx.render_context->uniform_sampler3D("volume_texture", 0);
        tv_3_volume_node->get_geometry()->bind_volume_texture(ctx, trilin_sampler_state_);

        ctx.render_context->bind_texture(volume_raycasting_front_buffer_color_result_, trilin_sampler_state_, 2);
        ctx.render_context->bind_texture(volume_raycasting_front_buffer_depth_result_, trilin_sampler_state_, 3);
        tv_3_volume_node->get_geometry()->apply_resource_dependent_uniforms(ctx, current_material_program);
        ctx.render_context->apply_texture_units();
        ctx.render_context->apply();

        ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST, 0, 36 * 6);

        ctx.render_context->reset_texture_units();
        // volume_raycasting_fbo_->clear_attachments();
        ++node_counter;

        ctx.render_context->set_frame_buffer(volume_compositing_fbo_);

        ctx.render_context->bind_texture(volume_raycasting_back_buffer_color_result_, trilin_sampler_state_, 0);
        ctx.render_context->bind_texture(volume_raycasting_front_buffer_color_result_, trilin_sampler_state_, 1);
        ctx.render_context->bind_texture(volume_raycasting_back_buffer_depth_result_, trilin_sampler_state_, 2);
        ctx.render_context->apply_texture_units();
        volume_compositing_shader_program_->use(ctx);
        volume_compositing_shader_program_->apply_uniform(ctx, "in_color_texture_current", 0);
        volume_compositing_shader_program_->apply_uniform(ctx, "in_color_texture_previous", 1);
        volume_compositing_shader_program_->apply_uniform(ctx, "in_depth_texture", 2);
        ctx.render_context->apply();
        fullscreen_quad_->draw(ctx.render_context);

        ctx.render_context->reset_texture_units();
    }

    // auto& gl_api = ctx.render_context->opengl_api();
    // gl_api.glDisable(GL_BLEND);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void TV_3VolumeRenderer::_postprocessing_pass(gua::Pipeline& pipe, PipelinePassDescription const& desc)
{
    RenderContext const& ctx(pipe.get_context());

    auto& target = *pipe.current_viewstate().target;
    bool write_depth = true;
    target.bind(ctx, write_depth);
    target.set_viewport(ctx);

    if(compositing_shader_program_ != nullptr)
    {
        compositing_shader_program_->use(ctx);
    }
    std::swap(volume_raycasting_front_buffer_color_result_, volume_raycasting_back_buffer_color_result_);
    pipe.get_gbuffer()->toggle_ping_pong();

    ctx.render_context->bind_texture(volume_raycasting_front_buffer_color_result_, trilin_sampler_state_, 0);
    ctx.render_context->bind_texture(pipe.get_gbuffer()->get_color_buffer(), trilin_sampler_state_, 1);
    compositing_shader_program_->apply_uniform(ctx, "blit_texture", 0);
    compositing_shader_program_->apply_uniform(ctx, "original_gbuffer_color", 1);

    ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);
    ctx.render_context->set_blend_state(no_blending_blend_state_);
    ctx.render_context->apply();
    fullscreen_quad_->draw(ctx.render_context);
    pipe.get_gbuffer()->toggle_ping_pong();
}

} // namespace gua
