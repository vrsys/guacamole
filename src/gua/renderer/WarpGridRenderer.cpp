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
#include <gua/renderer/WarpGridRenderer.hpp>

#include <gua/renderer/WarpPass.hpp>
#include <gua/config.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/WarpGridGenerator.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/Resources.hpp>

#include <scm/core/math/math.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

WarpGridRenderer::WarpGridRenderer()
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  ResourceFactory factory;
  std::string v_shader = factory.read_shader_file("shaders/warp_grid_shader.vert");
  std::string f_shader = factory.read_shader_file("shaders/warp_grid_shader.frag");
  std::string g_shader = factory.read_shader_file("shaders/warp_grid_shader.geom");
#else
  std::string v_shader = Resources::lookup_shader("shaders/warp_grid_shader.vert");
  std::string f_shader = Resources::lookup_shader("shaders/warp_grid_shader.frag");
  std::string g_shader = Resources::lookup_shader("shaders/warp_grid_shader.geom");
#endif

  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_shader));
  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
}

////////////////////////////////////////////////////////////////////////////////

void WarpGridRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{


  if (!program_) {
    program_ = std::make_shared<ShaderProgram>();
    program_->set_shaders(program_stages_, std::list<std::string>(), false);
  }

  auto& ctx(pipe.get_context());

  if (!depth_stencil_state_) {
    depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(
      false, false, scm::gl::COMPARISON_LESS, true, 1, 0,
      scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    );
    rasterizer_state_ = ctx.render_device->create_rasterizer_state(
      scm::gl::FILL_SOLID,
      scm::gl::CULL_NONE,
      scm::gl::ORIENT_CCW,
      false,
      false,
      0.0f,
      false,
      true,
      scm::gl::point_raster_state(true));
    blend_state_ = ctx.render_device->create_blend_state(scm::gl::blend_ops(true,
      scm::gl::FUNC_ONE,
      scm::gl::FUNC_ONE,
      scm::gl::FUNC_ONE,
      scm::gl::FUNC_ONE), false);
  }

  auto res(ctx.resources.get<WarpGridGenerator::SharedResource>());

  if (res) {
    std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpGridRenderer";
    std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpGridRenderer";
    std::string const pri_query_name = "Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpGridRenderer";

    pipe.begin_primitive_query(ctx, pri_query_name);
    pipe.begin_gpu_query(ctx, gpu_query_name);
    pipe.begin_cpu_query(cpu_query_name);

    auto& target = *pipe.current_viewstate().target;

    bool write_all_layers = false;
    target.bind(ctx, write_all_layers);
    target.set_viewport(ctx);

    program_->use(ctx);

    // ctx.render_context->set_blend_state(blend_state_);
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_, 1);
    ctx.render_context->bind_vertex_array(res->grid_vao[res->current_vbo()]);
    ctx.render_context->apply();
    ctx.render_context->draw_transform_feedback(scm::gl::PRIMITIVE_POINT_LIST, res->grid_tfb[res->current_vbo()]);

    target.unbind(ctx);

    pipe.end_primitive_query(ctx, pri_query_name);
    pipe.end_gpu_query(ctx, gpu_query_name);
    pipe.end_cpu_query(cpu_query_name);

    ctx.render_context->reset_state_objects();
  }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
