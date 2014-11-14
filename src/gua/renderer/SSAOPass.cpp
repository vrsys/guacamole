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
#include <gua/renderer/SSAOPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

SSAOPassDescription::SSAOPassDescription()
  : PipelinePassDescription() {

  vertex_shader_ = "shaders/common/fullscreen_quad.vert";
  fragment_shader_ = "shaders/ssao.frag";

  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = true;
  doClear_ = false;
  rendermode_ = RenderMode::Callback;

  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));

  blend_state_ = boost::make_optional(
      scm::gl::blend_state_desc(scm::gl::blend_ops(
                                            true,
                                            scm::gl::FUNC_SRC_ALPHA,
                                            scm::gl::FUNC_ONE_MINUS_SRC_ALPHA,
                                            scm::gl::FUNC_SRC_ALPHA,
                                            scm::gl::FUNC_ONE_MINUS_SRC_ALPHA)));
  uniforms1f["gua_ssao_radius"]    = 1.0f;
  uniforms1f["gua_ssao_intensity"] = 1.0f;
  uniforms1f["gua_ssao_falloff"]   = 0.1f;

  auto tex = std::make_shared<NoiseTexture>();

  // set uniforms and draw a full screen quad
  process_ = [tex](
      PipelinePass & pass, PipelinePassDescription* , Pipeline & pipe) {
    pipe.get_context().render_context->current_program()->uniform(
        "gua_noise_tex", 0, tex->get_handle(pipe.get_context()));

    pipe.draw_quad();
  };
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* SSAOPassDescription::make_copy() const {
  return new SSAOPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

}
