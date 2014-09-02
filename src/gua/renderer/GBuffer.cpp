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
#include <gua/renderer/GBuffer.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

GBuffer::GBuffer(RenderContext const& ctx, unsigned width, unsigned height) {

  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST,
    scm::gl::WRAP_MIRRORED_REPEAT,
    scm::gl::WRAP_MIRRORED_REPEAT);

  color_buffer_ = std::make_shared<Texture2D>(width, height,
                                              scm::gl::FORMAT_RGB_16UI,
                                              1, state);

  normal_buffer_ = std::make_shared<Texture2D>(width, height,
                                              scm::gl::FORMAT_RGB_16UI,
                                              1, state);

  depth_buffer_ = std::make_shared<Texture2D>(width, height,
                                              scm::gl::FORMAT_D24,
                                              1, state);

  attach_color_buffer(ctx, 0, color_buffer_);
  attach_color_buffer(ctx, 1, normal_buffer_);
  attach_depth_stencil_buffer(ctx, depth_buffer_);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::remove_buffers(RenderContext const& ctx) {
  unbind(ctx);
  remove_attachments();

  if (color_buffer_) {
    color_buffer_->make_non_resident(ctx);
  }
  if (normal_buffer_) {
    normal_buffer_->make_non_resident(ctx);
  }
  if (depth_buffer_) {
    depth_buffer_->make_non_resident(ctx);
  }

}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_color_buffer() const {
  return color_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_normal_buffer() const {
  return normal_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_depth_buffer() const {
  return depth_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

}
