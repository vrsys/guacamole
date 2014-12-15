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
#include <gua/renderer/FullscreenPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {

FullscreenPassDescription::FullscreenPassDescription()
  : PipelinePassDescription()
{
  vertex_shader_ = "shaders/common/fullscreen_quad.vert";
  fragment_shader_ = R"(
    @include "shaders/common/header.glsl"
    @include "shaders/common/gua_camera_uniforms.glsl"
    @include "shaders/common/gua_gbuffer_input.glsl"
    layout(location=0) out vec3 gua_out_color;
    void main() { 
      gua_out_color = gua_get_color();
    }
  )";

  Resources::resolve_includes(fragment_shader_);

  fragment_shader_is_file_name_ = false;
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = true;

  rendermode_ = RenderMode::Quad;

  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));
}

////////////////////////////////////////////////////////////////////////////////

FullscreenPassDescription& FullscreenPassDescription::source(std::string const& source) {
  fragment_shader_ = source;
  fragment_shader_is_file_name_ = false;
  Resources::resolve_includes(fragment_shader_);
  recompile_shaders_ = true;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& FullscreenPassDescription::source() const {
  return fragment_shader_;
}

////////////////////////////////////////////////////////////////////////////////

FullscreenPassDescription& FullscreenPassDescription::source_file(std::string const& source_file) {
  fragment_shader_ = source_file;
  fragment_shader_is_file_name_ = true;
  recompile_shaders_ = true;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& FullscreenPassDescription::source_file() const {
  return fragment_shader_;
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* FullscreenPassDescription::make_copy() const {
  return new FullscreenPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass FullscreenPassDescription::make_pass(RenderContext const& ctx) {
  PipelinePass pass{*this, ctx};
  return pass;
}

}
