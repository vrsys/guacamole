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
#include <gua/renderer/BackgroundPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {

BackgroundPassDescription::BackgroundPassDescription()
  : PipelinePassDescription()
{
  vertex_shader_ = "shaders/common/fullscreen_quad.vert";
  fragment_shader_ = "shaders/background.frag";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = true;
  rendermode_ = RenderMode::Quad;
  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));

  uniforms["background_color"]    = math::vec3(0.2f, 0.2f, 0.2f);
  uniforms["background_mode"]     = (int)COLOR;
  uniforms["background_texture"]  = std::string("gua_default_texture");
  uniforms["enable_fog"]          = false;
  uniforms["fog_start"]           = 10.f;
  uniforms["fog_end"]             = 1000.f;
}

////////////////////////////////////////////////////////////////////////////////

BackgroundPassDescription& BackgroundPassDescription::color(utils::Color3f const& color) {
  uniforms["background_color"] = color.vec3();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

utils::Color3f BackgroundPassDescription::color() const {
  auto uniform(uniforms.find("background_color"));
  return utils::Color3f(boost::get<math::vec3>(uniform->second.data));
}

////////////////////////////////////////////////////////////////////////////////

BackgroundPassDescription& BackgroundPassDescription::texture(std::string const& texture) {
  uniforms["background_texture"] = texture;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::string BackgroundPassDescription::texture() const {
  auto uniform(uniforms.find("background_texture"));
  return boost::get<std::string>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

BackgroundPassDescription& BackgroundPassDescription::mode(BackgroundPassDescription::BackgroundMode const& mode) {
  uniforms["background_mode"] = (int)mode;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

BackgroundPassDescription::BackgroundMode BackgroundPassDescription::mode() const {
  auto uniform(uniforms.find("background_mode"));
  return BackgroundPassDescription::BackgroundMode(boost::get<int>(uniform->second.data));
}

////////////////////////////////////////////////////////////////////////////////

BackgroundPassDescription& BackgroundPassDescription::enable_fog(bool enable_fog) {
  uniforms["enable_fog"] = enable_fog;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool BackgroundPassDescription::enable_fog() const {
  auto uniform(uniforms.find("enable_fog"));
  return boost::get<bool>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

BackgroundPassDescription& BackgroundPassDescription::fog_start(float fog_start) {
  uniforms["fog_start"] = fog_start;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

float BackgroundPassDescription::fog_start() const {
  auto uniform(uniforms.find("fog_start"));
  return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

BackgroundPassDescription& BackgroundPassDescription::fog_end(float fog_end) {
  uniforms["fog_end"] = fog_end;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

float BackgroundPassDescription::fog_end() const {
  auto uniform(uniforms.find("fog_end"));
  return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* BackgroundPassDescription::make_copy() const {
  return new BackgroundPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass BackgroundPassDescription::make_pass(RenderContext const& ctx)
{
  PipelinePass pass{*this, ctx};
  return pass;
}

}
