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
#include <gua/renderer/WarpRenderer.hpp>

#include <gua/renderer/WarpPass.hpp>
#include <gua/config.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/WarpGridGenerator.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/WindowDatabase.hpp>

#include <scm/core/math/math.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

WarpRenderer::WarpRenderer()
{}

////////////////////////////////////////////////////////////////////////////////

void WarpRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{

  auto& ctx(pipe.get_context());
  auto description(dynamic_cast<WarpPassDescription const*>(&desc));

  // ---------------------------------------------------------------------------
  // ------------------------------ allocate resources -------------------------
  // ---------------------------------------------------------------------------

  if (!warp_abuffer_program_) {
  #ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string v_shader = factory.read_shader_file("shaders/warp_abuffer.vert");
    std::string f_shader = factory.read_shader_file("shaders/warp_abuffer.frag");
    std::string g_shader = factory.read_shader_file("shaders/warp_abuffer.geom");
  #else
    std::string v_shader = Resources::lookup_shader("shaders/warp_abuffer.vert");
    std::string f_shader = Resources::lookup_shader("shaders/warp_abuffer.frag");
    std::string g_shader = Resources::lookup_shader("shaders/warp_abuffer.geom");
  #endif

    warp_abuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
    if (description->abuffer_warp_mode() != WarpPassDescription::ABUFFER_RAYCASTING) {
      warp_abuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_shader));
    }
    warp_abuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
    warp_abuffer_program_ = std::make_shared<ShaderProgram>();
    warp_abuffer_program_->set_shaders(warp_abuffer_program_stages_, std::list<std::string>(), false, global_substitution_map_);
  }

  if (!warp_gbuffer_program_) {
  #ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string v_shader = factory.read_shader_file("shaders/warp_gbuffer.vert");
    std::string f_shader = factory.read_shader_file("shaders/warp_gbuffer.frag");
    std::string g_shader = factory.read_shader_file("shaders/warp_gbuffer.geom");
  #else
    std::string v_shader = Resources::lookup_shader("shaders/warp_gbuffer.vert");
    std::string f_shader = Resources::lookup_shader("shaders/warp_gbuffer.frag");
    std::string g_shader = Resources::lookup_shader("shaders/warp_gbuffer.geom");
  #endif

    warp_gbuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
    if (description->gbuffer_warp_mode() != WarpPassDescription::GBUFFER_POINTS && description->gbuffer_warp_mode() != WarpPassDescription::GBUFFER_SCALED_POINTS) {
      warp_gbuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_shader));
    }
    warp_gbuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
    warp_gbuffer_program_ = std::make_shared<ShaderProgram>();
    warp_gbuffer_program_->set_shaders(warp_gbuffer_program_stages_, std::list<std::string>(), false, global_substitution_map_);
  }

  if (!points_) {
    scm::gl::rasterizer_state_desc p_desc;
    p_desc._point_state = scm::gl::point_raster_state(true);
    points_ = ctx.render_device->create_rasterizer_state(p_desc);

    depth_stencil_state_yes_ = ctx.render_device->create_depth_stencil_state(
        true, true, scm::gl::COMPARISON_LESS, true, 1, 0,
        scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    );

    depth_stencil_state_no_ = ctx.render_device->create_depth_stencil_state(
        false, false, scm::gl::COMPARISON_LESS, true, 1, 0,
        scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    );

    auto empty_vbo = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, sizeof(math::vec2), 0);
    empty_vao_ = ctx.render_device->create_vertex_array(scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC2F, sizeof(math::vec2)), {empty_vbo});
  }

  auto& target = *pipe.current_viewstate().target;

  bool write_all_layers = true;
  target.bind(ctx, write_all_layers);
  target.set_viewport(ctx);

  if (description->depth_test()) {
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_yes_, 1);
  } else {
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_no_, 1);
  }

  // get warp matrix -----------------------------------------------------------
  auto frustum(pipe.current_viewstate().frustum);
  if (ctx.mode != CameraMode::RIGHT) {
    cached_warp_state_ = description->get_warp_state()();
  }
  math::mat4f warp_matrix(cached_warp_state_.get(ctx.mode) * scm::math::inverse(gua::math::mat4f(frustum.get_projection() * frustum.get_view())));

  // ---------------------------------------------------------------------------
  // --------------------------------- warp gbuffer ----------------------------
  // ---------------------------------------------------------------------------

  std::string const gpu_query_name_a = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpPass GBuffer";
  std::string const pri_query_name_a = "Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpPass GBuffer";
  pipe.begin_gpu_query(ctx, gpu_query_name_a);
  pipe.begin_primitive_query(ctx, pri_query_name_a);

  if (description->gbuffer_warp_mode() != WarpPassDescription::GBUFFER_NONE) {
    warp_gbuffer_program_->use(ctx);
    warp_gbuffer_program_->apply_uniform(ctx, "warp_matrix", warp_matrix);

    pipe.bind_gbuffer_input(warp_gbuffer_program_);

    if (description->gbuffer_warp_mode() == WarpPassDescription::GBUFFER_GRID_DEPTH_THRESHOLD ||
        description->gbuffer_warp_mode() == WarpPassDescription::GBUFFER_GRID_SURFACE_ESTIMATION ||
        description->gbuffer_warp_mode() == WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION ||
        description->gbuffer_warp_mode() == WarpPassDescription::GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION) {
      auto res(ctx.resources.get_dont_create<WarpGridGenerator::SharedResource>());
      if (res) {
        ctx.render_context->bind_vertex_array(res->grid_vao[res->current_vbo()]);
        ctx.render_context->apply();
        ctx.render_context->draw_transform_feedback(scm::gl::PRIMITIVE_POINT_LIST, res->grid_tfb[res->current_vbo()]);
      }
    } else {
      ctx.render_context->set_rasterizer_state(points_);
      ctx.render_context->bind_vertex_array(empty_vao_);
      ctx.render_context->apply();
      math::vec2ui resolution(pipe.current_viewstate().camera.config.get_resolution());
      ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, resolution.x * resolution.y);
    }
  }

  pipe.end_primitive_query(ctx, pri_query_name_a);
  pipe.end_gpu_query(ctx, gpu_query_name_a);

  // ---------------------------------------------------------------------------
  // --------------------------------- warp abuffer ----------------------------
  // ---------------------------------------------------------------------------

  std::string const gpu_query_name_b = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpPass ABuffer";
  std::string const pri_query_name_b = "Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpPass ABuffer";
  pipe.begin_gpu_query(ctx, gpu_query_name_b);
  pipe.begin_primitive_query(ctx, pri_query_name_b);

  if (description->abuffer_warp_mode() != WarpPassDescription::ABUFFER_NONE) {
    warp_abuffer_program_->use(ctx);

    if (description->abuffer_warp_mode() == WarpPassDescription::ABUFFER_RAYCASTING) {
      warp_abuffer_program_->apply_uniform(ctx, "warp_matrix", scm::math::inverse(warp_matrix));
    } else {
      warp_abuffer_program_->apply_uniform(ctx, "warp_matrix", warp_matrix);
    }

    auto gbuffer = dynamic_cast<GBuffer*>(pipe.current_viewstate().target);
    warp_abuffer_program_->set_uniform(ctx, gbuffer->get_depth_buffer_write()->get_handle(ctx), "warped_depth_buffer");
    warp_abuffer_program_->set_uniform(ctx, gbuffer->get_color_buffer_write()->get_handle(ctx), "warped_color_buffer");
    warp_abuffer_program_->set_uniform(ctx, gbuffer->get_depth_buffer()->get_handle(ctx), "orig_depth_buffer");
    

    ABuffer a_buffer;
    a_buffer.allocate_shared(ctx);

    if (description->use_abuffer_from_window() != "") {
      auto shared_window(WindowDatabase::instance()->lookup(description->use_abuffer_from_window()));
      if (shared_window) {
        if (!shared_window->get_is_open()) {
          Logger::LOG_WARNING << "Failed to share ABuffer for WarpPass: Shared window is not opened yet!" << std::endl;
        } else {
          a_buffer.allocate_shared(*shared_window->get_context());
        }
      } else {
        Logger::LOG_WARNING << "Failed to share ABuffer for WarpPass: Target window not found!" << std::endl;
      }
    }

    a_buffer.bind(ctx);

    if (description->abuffer_warp_mode() == WarpPassDescription::ABUFFER_RAYCASTING) {
      ctx.render_context->set_depth_stencil_state(depth_stencil_state_no_, 1);
      ctx.render_context->apply();
      pipe.draw_quad();
    } else {
      ctx.render_context->set_rasterizer_state(points_);
      ctx.render_context->bind_vertex_array(empty_vao_);
      ctx.render_context->apply();
      math::vec2ui resolution(pipe.current_viewstate().camera.config.get_resolution());
      ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, resolution.x * resolution.y);
    }
  }

  pipe.end_primitive_query(ctx, pri_query_name_b);
  pipe.end_gpu_query(ctx, gpu_query_name_b);

  target.unbind(ctx);

  ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
