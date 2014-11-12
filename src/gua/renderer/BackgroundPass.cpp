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

namespace gua {

BackgroundPassDescription::BackgroundPassDescription()
  : PipelinePassDescription()
{
  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = true;
  rendermode_ = RenderMode::Quad;
  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));
}
////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* BackgroundPassDescription::make_copy() const {
  return new BackgroundPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass BackgroundPassDescription::make_pass(
    RenderContext const& ctx) const {
  PipelinePass pass{};

  pass.shader_ = std::make_shared<ShaderProgram>();
  pass.shader_->create_from_sources(
      Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_vert),
      Resources::lookup_shader(Resources::shaders_background_frag));

  pass.needs_color_buffer_as_input_ = needs_color_buffer_as_input_;
  pass.writes_only_color_buffer_    = writes_only_color_buffer_;
  pass.doClear_                     = doClear_;
  pass.rendermode_                  = rendermode_;

  if (depth_stencil_state_) {
    pass.depth_stencil_state_ =
        ctx.render_device->create_depth_stencil_state(*depth_stencil_state_);
  }
  if (blend_state_) {
    pass.blend_state_ = ctx.render_device->create_blend_state(*blend_state_);
  }
  if (rasterizer_state_) {
    pass.rasterizer_state_ =
      ctx.render_device->create_rasterizer_state(*rasterizer_state_);
  }

  return pass;
}

////////////////////////////////////////////////////////////////////////////////

}
