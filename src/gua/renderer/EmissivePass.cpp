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
#include <gua/renderer/EmissivePass.hpp>

namespace gua {

EmissivePassDescription::EmissivePassDescription()
  : PipelinePassDescription() {
  vertex_shader_ = "resources/shaders/lighting_emit.vert";
  fragment_shader_ = "resources/shaders/lighting_emit.frag";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = true;
  doClear_ = true;
  rendermode_ = RenderMode::Quad;

  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));
  blend_state_ = boost::make_optional(
      scm::gl::blend_state_desc(scm::gl::blend_ops(true,
                                                    scm::gl::FUNC_ONE,
                                                    scm::gl::FUNC_ONE,
                                                    scm::gl::FUNC_ONE,
                                                    scm::gl::FUNC_ONE)));
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* EmissivePassDescription::make_copy() const {
  return new EmissivePassDescription(*this);
}

PipelinePass EmissivePassDescription::make_pass(RenderContext const& ctx, SubstitutionMap const& substitution_map)
{
  PipelinePass pass{*this, ctx, substitution_map};
  return pass;
}

}
