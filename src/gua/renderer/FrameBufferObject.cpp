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
#include <gua/renderer/FrameBufferObject.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/math/math.hpp>

// external headers
#include <string>
#include <scm/gl_core/render_device/opengl/gl_core.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

FrameBufferObject::FrameBufferObject()
    : fbos_(), upload_mutex_(), width_(0), height_(0), sample_count_(0) {}

////////////////////////////////////////////////////////////////////////////////

FrameBufferObject::~FrameBufferObject() {

  remove_attachments();
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::initialize_with_no_attachments(RenderContext const& ctx,
                                                       unsigned width,
                                                       unsigned height,
                                                       unsigned sample_count) {
  std::unique_lock<std::mutex> lock(upload_mutex_);
  auto const& glapi = ctx.render_context->opengl_api();

  // create new fbo if there isn't any for this context
  if (fbos_.size() <= ctx.id || fbos_[ctx.id] == nullptr) {
    fbos_.resize(ctx.id + 1);
    fbos_[ctx.id] = ctx.render_device->create_frame_buffer();
  }

  // TODO: ideally, these should be implemented in schism
  if (width_ != width) {
    width_ = width;
    glapi.glNamedFramebufferParameteriEXT(fbos_[ctx.id]->object_id(),
                                          GL_FRAMEBUFFER_DEFAULT_WIDTH, width);
  }
  if (height_ != height) {
    height_ = height;
    glapi.glNamedFramebufferParameteriEXT(fbos_[ctx.id]->object_id(),
                                          GL_FRAMEBUFFER_DEFAULT_HEIGHT, height);
  }
  if (sample_count_ != sample_count) {
    sample_count_ = sample_count;
    glapi.glNamedFramebufferParameteriEXT(fbos_[ctx.id]->object_id(),
                                          GL_FRAMEBUFFER_DEFAULT_SAMPLES, sample_count);
  }
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::remove_attachments() {

  for (auto fbo: fbos_) {
    if (fbo){
      fbo->clear_attachments();
      fbo.reset();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::attach_color_buffer(RenderContext const& ctx,
                                            unsigned in_color_attachment,
                                            std::shared_ptr<Texture2D> const& buffer,
                                            int mip_level,
                                            int z_slice) {

  std::unique_lock<std::mutex> lock(upload_mutex_);

  // only attach buffer if it has an appropriate size
  if (set_size(buffer)) {

    // create new fbo if there isn't any for this context
    if (fbos_.size() <= ctx.id || fbos_[ctx.id] == nullptr) {
      fbos_.resize(ctx.id + 1);
      fbos_[ctx.id] = ctx.render_device->create_frame_buffer();
    }

    fbos_[ctx.id]->attach_color_buffer(
        in_color_attachment, buffer->get_buffer(ctx), mip_level, z_slice);
  }
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::attach_depth_stencil_buffer(
    RenderContext const& ctx,
    std::shared_ptr<Texture2D> const& buffer,
    int mip_level,
    int z_slice) {

  std::unique_lock<std::mutex> lock(upload_mutex_);

  // only attach buffer if it has an appropriate size
  if (set_size(buffer)) {

    // create new fbo if there isn't any for this context
    if (fbos_.size() <= ctx.id || fbos_[ctx.id] == nullptr) {
      fbos_.resize(ctx.id + 1);
      fbos_[ctx.id] = ctx.render_device->create_frame_buffer();
    }

    fbos_[ctx.id]->attach_depth_stencil_buffer(
        buffer->get_buffer(ctx), mip_level, z_slice);
  }
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::clear_color_buffers(RenderContext const& ctx,
                                            utils::Color3f const& color) {

  if (ctx.id < fbos_.size())
    ctx.render_context->clear_color_buffers(
        fbos_[ctx.id], math::vec4(color.r(), color.g(), color.b(), 0.f));
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::clear_color_buffer(RenderContext const& ctx,
                                           unsigned which,
                                           utils::Color3f const& color) {

  if (ctx.id < fbos_.size())
    ctx.render_context->clear_color_buffer(
        fbos_[ctx.id], which, math::vec4(color.r(), color.g(), color.b(), 0.f));
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::clear_depth_stencil_buffer(RenderContext const& ctx) {

  if (ctx.id < fbos_.size())
    ctx.render_context->clear_depth_stencil_buffer(fbos_[ctx.id]);
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::bind(RenderContext const& ctx) {

  if (ctx.id < fbos_.size())
    ctx.render_context->set_frame_buffer(fbos_[ctx.id]);
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::unbind(RenderContext const& ctx) {

  if (fbos_.size() > ctx.id && fbos_[ctx.id] != 0)
    ctx.render_context->reset_framebuffer();
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::copy_depth_stencil_buffer(
    RenderContext const& ctx,
    FrameBufferObject const& source) {

  ctx.render_context
      ->copy_depth_stencil_buffer(source.fbos_[ctx.id], fbos_[ctx.id]);
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::copy_color_buffer(RenderContext const& ctx,
                                          FrameBufferObject const& source,
                                          unsigned layer) {

  ctx.render_context
      ->copy_color_buffer(source.fbos_[ctx.id], fbos_[ctx.id], layer);
}

////////////////////////////////////////////////////////////////////////////////

void FrameBufferObject::set_viewport(RenderContext const& ctx) const {

  ctx.render_context->set_viewport(
    scm::gl::viewport(math::vec2(0.0f, 0.0f), math::vec2(float(width()), float(height()))));
}

////////////////////////////////////////////////////////////////////////////////

bool FrameBufferObject::set_size(std::shared_ptr<Texture2D> const& buffer) {

  if (width_ == 0 && height_ == 0) {
    width_ = buffer->width();
    height_ = buffer->height();
    return true;
  } else if (buffer->width() != width_ || buffer->height() != height_) {
    Logger::LOG_WARNING << "Buffers attached to the same FrameBufferObject must have the same size!" << std::endl;

    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////

}
