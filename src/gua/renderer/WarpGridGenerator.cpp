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
#include <gua/renderer/WarpGridGenerator.hpp>

#include <gua/renderer/Pipeline.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

WarpGridGenerator::WarpGridGenerator()
{
// #ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
//   ResourceFactory factory;
//   std::string v_shader = factory.read_shader_file("shaders/warp_shader.vert");
//   std::string f_shader = factory.read_shader_file("shaders/warp_shader.frag");
//   std::string g_shader = factory.read_shader_file("shaders/warp_shader.geom");
// #else
//   std::string v_shader = Resources::lookup_shader("shaders/warp_shader.vert");
//   std::string f_shader = Resources::lookup_shader("shaders/warp_shader.frag");
//   std::string g_shader = Resources::lookup_shader("shaders/warp_shader.geom");
// #endif

//   program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
//   program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_shader));
//   program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
}

////////////////////////////////////////////////////////////////////////////////

void WarpGridGenerator::render(Pipeline& pipe, PipelinePassDescription const& desc)
{
  auto const& ctx(pipe.get_context());

  std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpGridGenerator";
  std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpGridGenerator";
  std::string const pri_query_name = "Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpGridGenerator";

  pipe.begin_primitive_query(ctx, pri_query_name);
  pipe.begin_gpu_query(ctx, gpu_query_name);
  pipe.begin_cpu_query(cpu_query_name);

  if (!vao_) {
    auto empty_vbo = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, sizeof(math::vec2), 0);
    vao_ = ctx.render_device->create_vertex_array(scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC2F, sizeof(math::vec2)), {empty_vbo});
  }

  pipe.end_primitive_query(ctx, pri_query_name);
  pipe.end_gpu_query(ctx, gpu_query_name);
  pipe.end_cpu_query(cpu_query_name);

  ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
