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

#include <gua/config.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

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

  if (!points_) {
    scm::gl::rasterizer_state_desc desc;
    desc._point_state = scm::gl::point_raster_state(true);
    points_ = ctx.render_device->create_rasterizer_state(desc);

    depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(
        true, true, scm::gl::COMPARISON_LESS, true, 1, 0,
        scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    );
  }

  auto& target = *pipe.current_viewstate().target;

  target.bind(ctx, true);
  target.set_viewport(ctx);
  // if (blend_state_)
  //   ctx.render_context->set_blend_state(blend_state_);
  // if (rasterizer_state_)
  ctx.render_context->set_depth_stencil_state(depth_stencil_state_, 1);
  ctx.render_context->set_rasterizer_state(points_);
  program_->use(ctx);

  for (auto const& u : desc.uniforms) {
    u.second.apply(ctx, u.first, ctx.render_context->current_program(), 0);
  }

  pipe.bind_gbuffer_input(program_);
  pipe.bind_light_table(program_);

  ctx.render_context->apply();

  ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, 1920*1080);

  target.unbind(ctx);
  ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
