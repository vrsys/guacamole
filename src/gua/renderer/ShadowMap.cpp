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
#include <gua/renderer/ShadowMap.hpp>

// guacamole headers

#include <gua/node/LightNode.hpp>
#include <gua/renderer/Serializer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/View.hpp>
#include <gua/databases.hpp>
#include <gua/memory.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

ShadowMap::ShadowMap(RenderContext const& ctx, math::vec2ui const& resolution)
    : RenderTarget(resolution), fbo_(nullptr), depth_buffer_(nullptr), viewport_offset_(math::vec2f(0.f, 0.f)), viewport_size_(math::vec2f(resolution))
{
    scm::gl::sampler_state_desc sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR,
                                                   // scm::gl::FILTER_ANISOTROPIC,
                                                   scm::gl::WRAP_CLAMP_TO_EDGE,
                                                   scm::gl::WRAP_CLAMP_TO_EDGE);
    sampler_state_desc._compare_mode = scm::gl::TEXCOMPARE_COMPARE_REF_TO_TEXTURE;
    sampler_state_desc._max_anisotropy = 16;

    sampler_state_ = ctx.render_device->create_sampler_state(sampler_state_desc);

    depth_buffer_ = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_D16, 1);
    ctx.render_context->make_resident(depth_buffer_, sampler_state_);

    fbo_ = ctx.render_device->create_frame_buffer();
    fbo_->attach_depth_stencil_buffer(depth_buffer_, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::clear(RenderContext const& ctx, float depth, unsigned stencil) { ctx.render_context->clear_depth_stencil_buffer(fbo_, depth, stencil); }

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::bind(RenderContext const& ctx, bool write_depth) { ctx.render_context->set_frame_buffer(fbo_); }

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::set_viewport(RenderContext const& ctx)
{
    if(ctx.render_context)
    {
        ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2f(viewport_size_.x * viewport_offset_.x, viewport_size_.y * viewport_offset_.y), scm::math::vec2f(viewport_size_)));
    }
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::set_viewport_offset(math::vec2f const& offset) { viewport_offset_ = offset; }

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::set_viewport_size(math::vec2f const& size) { viewport_size_ = size; }

////////////////////////////////////////////////////////////////////////////////

scm::gl::texture_2d_ptr const& ShadowMap::get_depth_buffer() const { return depth_buffer_; }

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::remove_buffers(RenderContext const& ctx)
{
    unbind(ctx);

    fbo_->clear_attachments();

    if(depth_buffer_)
    {
        ctx.render_context->make_non_resident(depth_buffer_);
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
