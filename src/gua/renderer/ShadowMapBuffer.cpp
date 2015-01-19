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
#include <gua/renderer/ShadowMapBuffer.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

ShadowMapBuffer::ShadowMapBuffer(RenderContext const& ctx, math::vec2ui const& resolution):
  //fbo_(new FrameBufferObject()),
  width_(resolution.x),
  height_(resolution.y) {

  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST,
    scm::gl::WRAP_MIRRORED_REPEAT,
    scm::gl::WRAP_MIRRORED_REPEAT);

  depth_buffer_ = std::make_shared<Texture2D>(width_, height_, scm::gl::FORMAT_D24, 1, state);
  //fbo_->attach_depth_stencil_buffer(ctx, depth_buffer_);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMapBuffer::clear(RenderContext const& ctx) {
  //fbo_->clear_depth_stencil_buffer(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMapBuffer::set_viewport(RenderContext const& ctx) {
  //fbo_->set_viewport(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMapBuffer::bind(RenderContext const& ctx) {
  //fbo_->bind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMapBuffer::unbind(RenderContext const& ctx) {
  //fbo_->unbind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMapBuffer::remove_buffers(RenderContext const& ctx) {
  //fbo_->unbind(ctx);
  //fbo_->remove_attachments();

  if (depth_buffer_) {
    depth_buffer_->make_non_resident(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& ShadowMapBuffer::get_depth_buffer() const {
  return depth_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

}
