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
#include <gua/renderer/ToneMappingPass.hpp>

#include <gua/renderer/Pipeline.hpp>

namespace gua {

ToneMappingPassDescription::ToneMappingPassDescription()
  : PipelinePassDescription()
{
  vertex_shader_ = "shaders/common/fullscreen_quad.vert";
  fragment_shader_ = "shaders/tone_mapping.frag";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = true;
  rendermode_ = RenderMode::Quad;
  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));
  uniforms["exposure"] = 1.0f;
  uniforms["selected_operator"] = static_cast<int>(Method::LINEAR);
}

ToneMappingPassDescription& ToneMappingPassDescription::exposure(float e) {
  uniforms["exposure"] = e;
  return *this;
}

float ToneMappingPassDescription::exposure() const {
  auto uniform(uniforms.find("exposure"));
  return boost::get<float>(uniform->second.data);
}

ToneMappingPassDescription& ToneMappingPassDescription::method(
    ToneMappingPassDescription::Method m) {
  uniforms["selected_operator"] = static_cast<int>(m);
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

ToneMappingPassDescription::Method ToneMappingPassDescription::method() const {
  auto uniform(uniforms.find("selected_operator"));
  return ToneMappingPassDescription::Method(
      boost::get<int>(uniform->second.data));
}

}
