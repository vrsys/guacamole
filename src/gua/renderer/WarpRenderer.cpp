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
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  ResourceFactory factory;
  std::string v_shader = factory.read_shader_file("shaders/warp_shader.vert");
  std::string f_shader = factory.read_shader_file("shaders/warp_shader.frag");
  std::string g_shader = factory.read_shader_file("shaders/warp_shader.geom");
#else
  std::string v_shader = Resources::lookup_shader("shaders/warp_shader.vert");
  std::string f_shader = Resources::lookup_shader("shaders/warp_shader.frag");
  std::string g_shader = Resources::lookup_shader("shaders/warp_shader.geom");
#endif

  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_shader));
  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
}

////////////////////////////////////////////////////////////////////////////////

void WarpRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{

  if (!program_) {
    program_ = std::make_shared<ShaderProgram>();
    program_->set_shaders(program_stages_, std::list<std::string>(), false, global_substitution_map_);
  }

  auto const& ctx(pipe.get_context());

  auto description(dynamic_cast<WarpPassDescription const*>(&desc));

  std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpPass";
  std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpPass";
  std::string const pri_query_name = "Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpPass";

  pipe.begin_primitive_query(ctx, pri_query_name);
  pipe.begin_gpu_query(ctx, gpu_query_name);
  pipe.begin_cpu_query(cpu_query_name);

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

  bool write_depth = false;
  target.bind(ctx, write_depth);
  target.set_viewport(ctx);

  if (description->depth_test()) ctx.render_context->set_depth_stencil_state(depth_stencil_state_yes_, 1);
  else ctx.render_context->set_depth_stencil_state(depth_stencil_state_no_, 1);
  ctx.render_context->set_rasterizer_state(points_);
  program_->use(ctx);

  for (auto const& u : desc.uniforms) {
    u.second.apply(ctx, u.first, ctx.render_context->current_program(), 0);
  }

  ABuffer a_buffer;
  a_buffer.allocate_shared(ctx);

  auto warp_desc(dynamic_cast<WarpPassDescription const*>(&desc));
  if (warp_desc->use_abuffer_from_window() != "") {
    auto shared_window(WindowDatabase::instance()->lookup(warp_desc->use_abuffer_from_window()));
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

  ctx.render_context->bind_vertex_array(empty_vao_);
  ctx.render_context->apply();

  math::vec2ui resolution(pipe.current_viewstate().camera.config.get_resolution());
  int pixel_count(resolution.x * resolution.y);

  ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, pixel_count);

  target.unbind(ctx);

  pipe.end_primitive_query(ctx, pri_query_name);
  pipe.end_gpu_query(ctx, gpu_query_name);
  pipe.end_cpu_query(cpu_query_name);

  ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
