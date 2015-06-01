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
  RenderTarget(resolution),
  abuffer_(),
  fbo_read_(nullptr),
  fbo_write_(nullptr),
  fbo_read_only_color_(nullptr),
  fbo_write_only_color_(nullptr) {

#if 0
  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST,
    scm::gl::WRAP_MIRRORED_REPEAT,
    scm::gl::WRAP_MIRRORED_REPEAT);
#else
  // linear filtering, only necessary for SSAA 3.11
  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_LINEAR,
    scm::gl::WRAP_MIRRORED_REPEAT,
    scm::gl::WRAP_MIRRORED_REPEAT);
#endif

  color_buffer_r_       = std::make_shared<Texture2D>(resolution.x, resolution.y, scm::gl::FORMAT_RGB_32F, 1, state);
  color_buffer_w_       = std::make_shared<Texture2D>(resolution.x, resolution.y, scm::gl::FORMAT_RGB_32F, 1, state);
  pbr_buffer_r_         = std::make_shared<Texture2D>(resolution.x, resolution.y, scm::gl::FORMAT_RGB_8,   1, state);
  pbr_buffer_w_         = std::make_shared<Texture2D>(resolution.x, resolution.y, scm::gl::FORMAT_RGB_8,   1, state);
  normal_buffer_r_      = std::make_shared<Texture2D>(resolution.x, resolution.y, scm::gl::FORMAT_RGB_16,  1, state);
  normal_buffer_w_      = std::make_shared<Texture2D>(resolution.x, resolution.y, scm::gl::FORMAT_RGB_16,  1, state);
  depth_buffer_r_       = std::make_shared<Texture2D>(resolution.x, resolution.y, scm::gl::FORMAT_D24_S8,  1, state);
  depth_buffer_w_       = std::make_shared<Texture2D>(resolution.x, resolution.y, scm::gl::FORMAT_D24_S8,  1, state);

  fbo_read_ = ctx.render_device->create_frame_buffer();
  fbo_read_->attach_color_buffer(0, color_buffer_r_->get_buffer(ctx),0,0);
  fbo_read_->attach_color_buffer(1, pbr_buffer_r_->get_buffer(ctx), 0, 0);
  fbo_read_->attach_color_buffer(2, normal_buffer_r_->get_buffer(ctx),0,0);
  fbo_read_->attach_depth_stencil_buffer(depth_buffer_r_->get_buffer(ctx),0,0);

  fbo_write_ = ctx.render_device->create_frame_buffer();
  fbo_write_->attach_color_buffer(0, color_buffer_w_->get_buffer(ctx),0,0);
  fbo_write_->attach_color_buffer(1, pbr_buffer_w_->get_buffer(ctx),0,0);
  fbo_write_->attach_color_buffer(2, normal_buffer_w_->get_buffer(ctx),0,0);
  fbo_write_->attach_depth_stencil_buffer(depth_buffer_w_->get_buffer(ctx),0,0);

  fbo_read_only_color_ = ctx.render_device->create_frame_buffer();
  fbo_read_only_color_->attach_color_buffer(0, color_buffer_r_->get_buffer(ctx),0,0);
  fbo_read_only_color_->attach_depth_stencil_buffer(depth_buffer_r_->get_buffer(ctx),0,0);

  fbo_write_only_color_ = ctx.render_device->create_frame_buffer();
  fbo_write_only_color_->attach_color_buffer(0, color_buffer_w_->get_buffer(ctx),0,0);
  fbo_write_only_color_->attach_depth_stencil_buffer(depth_buffer_w_->get_buffer(ctx),0,0);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::allocate_a_buffer(RenderContext& ctx, size_t buffer_size) {
  abuffer_.allocate(ctx, buffer_size);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::clear(RenderContext const& ctx, float depth, unsigned stencil) {
  ctx.render_context->clear_color_buffers(
      fbo_write_, scm::math::vec4f(0,0,0,0));
  ctx.render_context->clear_depth_stencil_buffer(fbo_write_, depth, stencil);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::clear_abuffer(RenderContext const& ctx) {
  abuffer_.clear(ctx, get_resolution());
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::bind(RenderContext const& ctx, bool write_all_layers) {
  if (write_all_layers) {
    ctx.render_context->set_frame_buffer(fbo_write_);
  } else {
    ctx.render_context->set_frame_buffer(fbo_write_only_color_);
  }
  abuffer_.bind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::unbind(RenderContext const& ctx) {
  abuffer_.unbind(ctx);
  RenderTarget::unbind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::toggle_ping_pong() {
  std::swap(fbo_write_, fbo_read_);
  std::swap(fbo_write_only_color_, fbo_read_only_color_);
  std::swap(color_buffer_w_, color_buffer_r_);
  std::swap(pbr_buffer_w_, pbr_buffer_r_);
  std::swap(normal_buffer_w_, normal_buffer_r_);
  std::swap(depth_buffer_w_, depth_buffer_r_);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::remove_buffers(RenderContext const& ctx) {
  unbind(ctx);
  
  if (fbo_write_) {
    fbo_write_->clear_attachments();
    fbo_write_.reset();
  }

  if (fbo_read_) {
    fbo_read_->clear_attachments();
    fbo_read_.reset();
  }

  if (color_buffer_w_) {
    color_buffer_w_->make_non_resident(ctx);
  }
  if (color_buffer_r_) {
    color_buffer_r_->make_non_resident(ctx);
  }
  if (pbr_buffer_r_) {
    pbr_buffer_r_->make_non_resident(ctx);
  }
  if (pbr_buffer_w_) {
    pbr_buffer_w_->make_non_resident(ctx);
  }
  if (normal_buffer_r_) {
    normal_buffer_r_->make_non_resident(ctx);
  }
  if (normal_buffer_w_) {
    normal_buffer_w_->make_non_resident(ctx);
  }
  if (depth_buffer_r_) {
    depth_buffer_r_->make_non_resident(ctx);
  }
  if (depth_buffer_w_) {
    depth_buffer_w_->make_non_resident(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_color_buffer() const {
  return color_buffer_r_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_pbr_buffer() const {
  return pbr_buffer_r_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_normal_buffer() const {
  return normal_buffer_r_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_depth_buffer() const {
  return depth_buffer_r_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_color_buffer_write() const {
  return color_buffer_w_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_pbr_buffer_write() const {
  return pbr_buffer_w_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_normal_buffer_write() const {
  return normal_buffer_w_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& GBuffer::get_depth_buffer_write() const {
  return depth_buffer_w_;
}

////////////////////////////////////////////////////////////////////////////////

}
