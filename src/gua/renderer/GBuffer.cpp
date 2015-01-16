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

GBuffer::GBuffer(RenderContext const& ctx, math::vec2ui const& resolution):
  fbo_read_(nullptr),
  fbo_write_(nullptr),
  fbo_read_only_color_(nullptr),
  fbo_write_only_color_(nullptr),
  width_(resolution.x),
  height_(resolution.y) {

  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST,
    scm::gl::WRAP_MIRRORED_REPEAT,
    scm::gl::WRAP_MIRRORED_REPEAT);

  color_buffer_read_  = std::make_shared<Texture2D>(width_, height_, scm::gl::FORMAT_RGB_32F, 1, state);
  color_buffer_write_ = std::make_shared<Texture2D>(width_, height_, scm::gl::FORMAT_RGB_32F, 1, state);
  pbr_buffer_         = std::make_shared<Texture2D>(width_, height_, scm::gl::FORMAT_RGB_8, 1, state);
  normal_buffer_      = std::make_shared<Texture2D>(width_, height_, scm::gl::FORMAT_RGB_16, 1, state);
  flags_buffer_       = std::make_shared<Texture2D>(width_, height_, scm::gl::FORMAT_R_8UI, 1, state);
  depth_buffer_       = std::make_shared<Texture2D>(width_, height_, scm::gl::FORMAT_D24, 1, state);

  fbo_read_ = ctx.render_device->create_frame_buffer();
  fbo_read_->attach_color_buffer(0, color_buffer_read_->get_buffer(ctx),0,0);
  fbo_read_->attach_color_buffer(1, pbr_buffer_->get_buffer(ctx), 0, 0);
  fbo_read_->attach_color_buffer(2, normal_buffer_->get_buffer(ctx),0,0);
  fbo_read_->attach_color_buffer(3, flags_buffer_->get_buffer(ctx),0,0);
  fbo_read_->attach_depth_stencil_buffer(depth_buffer_->get_buffer(ctx),0,0);

  fbo_write_ = ctx.render_device->create_frame_buffer();
  fbo_write_->attach_color_buffer(0, color_buffer_write_->get_buffer(ctx),0,0);
  fbo_write_->attach_color_buffer(1, pbr_buffer_->get_buffer(ctx),0,0);
  fbo_write_->attach_color_buffer(2, normal_buffer_->get_buffer(ctx),0,0);
  fbo_write_->attach_color_buffer(3, flags_buffer_->get_buffer(ctx),0,0);
  fbo_write_->attach_depth_stencil_buffer(depth_buffer_->get_buffer(ctx),0,0);

  fbo_read_only_color_ = ctx.render_device->create_frame_buffer();
  fbo_read_only_color_->attach_color_buffer(0, color_buffer_read_->get_buffer(ctx),0,0);
  fbo_read_only_color_->attach_depth_stencil_buffer(depth_buffer_->get_buffer(ctx),0,0);

  fbo_write_only_color_ = ctx.render_device->create_frame_buffer();
  fbo_write_only_color_->attach_color_buffer(0, color_buffer_write_->get_buffer(ctx),0,0);
  fbo_write_only_color_->attach_depth_stencil_buffer(depth_buffer_->get_buffer(ctx),0,0);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::clear_all(RenderContext const& ctx) {
  ctx.render_context->clear_color_buffers(
      fbo_write_, math::vec4(0, 0, 0, 0.f));
  ctx.render_context->clear_depth_stencil_buffer(fbo_write_);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::clear_color(RenderContext const& ctx) {
  if (ctx.render_context && fbo_write_)
    ctx.render_context->clear_color_buffer(
        fbo_write_, 0, math::vec4(0, 0, 0, 0.f));
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::set_viewport(RenderContext const& ctx) {
  if (ctx.render_context)
    ctx.render_context->set_viewport(
        scm::gl::viewport(math::vec2(0.0f, 0.0f),
        math::vec2(float(width_), float(height_))));
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::bind(RenderContext const& ctx, bool pass_writes_only_color_buffer) {
  if (pass_writes_only_color_buffer) {
    ctx.render_context->set_frame_buffer(fbo_write_only_color_);
  } else {
    ctx.render_context->set_frame_buffer(fbo_write_);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::unbind(RenderContext const& ctx) {
  ctx.render_context->reset_framebuffer();
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::toggle_ping_pong() {
  std::swap(fbo_write_, fbo_read_);
  std::swap(fbo_write_only_color_, fbo_read_only_color_);
  std::swap(color_buffer_write_, color_buffer_read_);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::remove_buffers(RenderContext const& ctx) {
  ctx.render_context->reset_framebuffer();
  if (fbo_write_) {
    fbo_write_->clear_attachments();
    fbo_write_.reset();
  }

  if (fbo_read_) {
    fbo_read_->clear_attachments();
    fbo_read_.reset();
  }

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
  if (flags_buffer_) {
    flags_buffer_->make_non_resident(ctx);
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

std::shared_ptr<Texture2D> const& GBuffer::get_current_flags_buffer() const {
  return flags_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_current_depth_buffer() const {
  return depth_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

}
