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

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

GBuffer::GBuffer(RenderContext const& ctx, math::vec2ui const& resolution)
    : RenderTarget(resolution), abuffer_(), fbo_read_(nullptr), fbo_write_(nullptr), fbo_read_only_color_(nullptr), fbo_write_only_color_(nullptr),
      sampler_state_desc_(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT)
// linear filtering, only necessary for SSAA 3.11
// sampler_state_desc_(scm::gl::FILTER_MIN_MAG_NEAREST,
//  scm::gl::WRAP_MIRRORED_REPEAT,
//  scm::gl::WRAP_MIRRORED_REPEAT)
{
    sampler_state_ = ctx.render_device->create_sampler_state(sampler_state_desc_);

    color_buffer_read_ = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
    ctx.render_context->make_resident(color_buffer_read_, sampler_state_);
    color_buffer_write_ = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
    ctx.render_context->make_resident(color_buffer_write_, sampler_state_);

    pbr_buffer_ = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_8, 1);
    ctx.render_context->make_resident(pbr_buffer_, sampler_state_);
    normal_buffer_ = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_16, 1);
    ctx.render_context->make_resident(normal_buffer_, sampler_state_);
    flags_buffer_ = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_R_8UI, 1);
    ctx.render_context->make_resident(flags_buffer_, sampler_state_);

    depth_buffer_ = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
    ctx.render_context->make_resident(depth_buffer_, sampler_state_);

    fbo_read_ = ctx.render_device->create_frame_buffer();
    fbo_read_->attach_color_buffer(0, color_buffer_read_, 0, 0);
    fbo_read_->attach_color_buffer(1, pbr_buffer_, 0, 0);
    fbo_read_->attach_color_buffer(2, normal_buffer_, 0, 0);
    fbo_read_->attach_color_buffer(3, flags_buffer_, 0, 0);
    fbo_read_->attach_depth_stencil_buffer(depth_buffer_, 0, 0);

    fbo_write_ = ctx.render_device->create_frame_buffer();
    fbo_write_->attach_color_buffer(0, color_buffer_write_, 0, 0);
    fbo_write_->attach_color_buffer(1, pbr_buffer_, 0, 0);
    fbo_write_->attach_color_buffer(2, normal_buffer_, 0, 0);
    fbo_write_->attach_color_buffer(3, flags_buffer_, 0, 0);
    fbo_write_->attach_depth_stencil_buffer(depth_buffer_, 0, 0);


    fbo_read_only_color_ = ctx.render_device->create_frame_buffer();
    fbo_read_only_color_->attach_color_buffer(0, color_buffer_read_, 0, 0);
    fbo_read_only_color_->attach_depth_stencil_buffer(depth_buffer_, 0, 0);

    fbo_write_only_color_ = ctx.render_device->create_frame_buffer();
    fbo_write_only_color_->attach_color_buffer(0, color_buffer_write_, 0, 0);
    fbo_write_only_color_->attach_depth_stencil_buffer(depth_buffer_, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::allocate_a_buffer(RenderContext& ctx, size_t buffer_size) { abuffer_.allocate(ctx, buffer_size); }

////////////////////////////////////////////////////////////////////////////////

void GBuffer::clear(RenderContext const& ctx, float depth, unsigned stencil)
{
    ctx.render_context->clear_color_buffers(fbo_write_, scm::math::vec4f(0, 0, 0, 0));
    ctx.render_context->clear_depth_stencil_buffer(fbo_write_, depth, stencil);

    abuffer_.clear(ctx, get_resolution());
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::clear_color(RenderContext const& ctx)
{
    if(ctx.render_context && fbo_write_)
        ctx.render_context->clear_color_buffer(fbo_write_, 0, scm::math::vec4f(0, 0, 0, 0));
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::bind(RenderContext const& ctx, bool write_depth)
{
    if(write_depth)
    {
        ctx.render_context->set_frame_buffer(fbo_write_);
    }
    else
    {
        ctx.render_context->set_frame_buffer(fbo_write_only_color_);
    }
    abuffer_.bind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::unbind(RenderContext const& ctx)
{
    abuffer_.unbind(ctx);
    RenderTarget::unbind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::toggle_ping_pong()
{
    std::swap(fbo_write_, fbo_read_);
    std::swap(fbo_write_only_color_, fbo_read_only_color_);
    std::swap(color_buffer_write_, color_buffer_read_);
}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::remove_buffers(RenderContext const& ctx)
{
    unbind(ctx);

    if(fbo_write_)
    {
        fbo_write_->clear_attachments();
        fbo_write_.reset();
    }

    if(fbo_read_)
    {
        fbo_read_->clear_attachments();
        fbo_read_.reset();
    }

    if(color_buffer_write_)
    {
        ctx.render_context->make_non_resident(color_buffer_write_);
    }
    if(color_buffer_read_)
    {
        ctx.render_context->make_non_resident(color_buffer_read_);
    }
    if(pbr_buffer_)
    {
        ctx.render_context->make_non_resident(pbr_buffer_);
    }
    if(normal_buffer_)
    {
        ctx.render_context->make_non_resident(normal_buffer_);
    }
    if(flags_buffer_)
    {
        ctx.render_context->make_non_resident(flags_buffer_);
    }
    if(depth_buffer_)
    {
        ctx.render_context->make_non_resident(depth_buffer_);
    }

}

////////////////////////////////////////////////////////////////////////////////

void GBuffer::retrieve_depth_data(RenderContext const& ctx, uint32_t* out_data)
{
    toggle_ping_pong();
    ctx.render_context->retrieve_texture_data(depth_buffer_, 0, out_data);
    toggle_ping_pong();
}
////////////////////////////////////////////////////////////////////////////////

} // namespace gua
