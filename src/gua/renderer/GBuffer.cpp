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
#include <gua/renderer/PipelinePass.hpp>
#include <gua/platform.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

GBuffer::GBuffer(RenderContext const& ctx, unsigned width, unsigned height):
  fbo_read_(new FrameBufferObject()),
  fbo_write_(new FrameBufferObject()),
  fbo_read_only_color_(new FrameBufferObject()),
  fbo_write_only_color_(new FrameBufferObject()),
  width_(width),
  height_(height) {

  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST,
    scm::gl::WRAP_MIRRORED_REPEAT,
    scm::gl::WRAP_MIRRORED_REPEAT);

  color_buffer_read_  = std::make_shared<Texture2D>(width, height, scm::gl::FORMAT_RGB_8, 1, state);
  color_buffer_write_ = std::make_shared<Texture2D>(width, height, scm::gl::FORMAT_RGB_8, 1, state);
  pbr_buffer_         = std::make_shared<Texture2D>(width, height, scm::gl::FORMAT_RGB_8, 1, state);
  normal_buffer_      = std::make_shared<Texture2D>(width, height, scm::gl::FORMAT_RGB_16, 1, state);
  depth_buffer_       = std::make_shared<Texture2D>(width, height, scm::gl::FORMAT_D24, 1, state);

  fbo_read_->attach_color_buffer(ctx, 0, color_buffer_read_);
  fbo_read_->attach_color_buffer(ctx, 1, pbr_buffer_);
  fbo_read_->attach_color_buffer(ctx, 2, normal_buffer_);
  fbo_read_->attach_depth_stencil_buffer(ctx, depth_buffer_);

  fbo_write_->attach_color_buffer(ctx, 0, color_buffer_write_);
  fbo_write_->attach_color_buffer(ctx, 1, pbr_buffer_);
  fbo_write_->attach_color_buffer(ctx, 2, normal_buffer_);
  fbo_write_->attach_depth_stencil_buffer(ctx, depth_buffer_);
  
  fbo_read_only_color_->attach_color_buffer(ctx, 0, color_buffer_read_);
  fbo_read_only_color_->attach_depth_stencil_buffer(ctx, depth_buffer_);

  fbo_write_only_color_->attach_color_buffer(ctx, 0, color_buffer_write_);
  fbo_write_only_color_->attach_depth_stencil_buffer(ctx, depth_buffer_);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::clear_all(RenderContext const& ctx) {
  fbo_write_->clear_color_buffers(ctx, utils::Color3f(0, 0, 0));
  fbo_write_->clear_depth_stencil_buffer(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::clear_color(RenderContext const& ctx) {
  fbo_write_->clear_color_buffer(ctx, 0, utils::Color3f(0, 0, 0));
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::set_viewport(RenderContext const& ctx) {
  fbo_write_->set_viewport(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::bind(RenderContext const& ctx, PipelinePass* next_pass) {
  if (next_pass->writes_only_color_buffer()) {
    fbo_write_only_color_->bind(ctx);
  } else {
    fbo_write_->bind(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::unbind(RenderContext const& ctx) {
  fbo_write_->unbind(ctx);
  fbo_write_only_color_->unbind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::toggle_ping_pong() {
  std::swap(fbo_write_, fbo_read_);
  std::swap(fbo_write_only_color_, fbo_read_only_color_);
  std::swap(color_buffer_write_, color_buffer_read_);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::remove_buffers(RenderContext const& ctx) {
  fbo_write_->unbind(ctx);
  fbo_write_->remove_attachments();

  fbo_read_->unbind(ctx);
  fbo_read_->remove_attachments();

  if (color_buffer_write_) {
    color_buffer_write_->make_non_resident(ctx);
  }
  if (color_buffer_read_) {
    color_buffer_read_->make_non_resident(ctx);
  }
  if (pbr_buffer_) {
    pbr_buffer_->make_non_resident(ctx);
  }
  if (normal_buffer_) {
    normal_buffer_->make_non_resident(ctx);
  }
  if (depth_buffer_) {
    depth_buffer_->make_non_resident(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_current_color_buffer() const {
  return color_buffer_read_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_current_pbr_buffer() const {
  return pbr_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_current_normal_buffer() const {
  return normal_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_current_depth_buffer() const {
  return depth_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

}
