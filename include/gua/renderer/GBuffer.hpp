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

#ifndef GUA_GBUFFER_HPP
#define GUA_GBUFFER_HPP

// guacamole headers
#include <gua/renderer/RenderTarget.hpp>
#include <gua/renderer/ABuffer.hpp>

#include <memory>

namespace gua
{
class GUA_DLL GBuffer : public RenderTarget
{
  public:
    GBuffer(RenderContext const& ctx, math::vec2ui const& resolution);

    void clear(RenderContext const& context, float depth = 1.f, unsigned stencil = 0) override;
    void clear_color(RenderContext const& context);

    void bind(RenderContext const& context, bool write_depth) override;
    void unbind(RenderContext const& context) override;

    void toggle_ping_pong();

    void allocate_a_buffer(RenderContext& ctx, size_t buffer_size);
    void remove_buffers(RenderContext const& ctx) override;

    void retrieve_depth_data(RenderContext const& ctx, uint32_t* out_data);

    inline scm::gl::texture_2d_ptr const& get_color_buffer() const { return color_buffer_read_; }
    inline scm::gl::texture_2d_ptr const& get_pbr_buffer() const { return pbr_buffer_; }
    inline scm::gl::texture_2d_ptr const& get_normal_buffer() const { return normal_buffer_; }
    inline scm::gl::texture_2d_ptr const& get_flags_buffer() const { return flags_buffer_; }
    inline scm::gl::texture_2d_ptr const& get_depth_buffer() const override { return depth_buffer_; }

    inline scm::gl::frame_buffer_ptr get_fbo_read() const { return fbo_read_; }
    inline scm::gl::sampler_state_desc const& get_sampler_state_desc() const { return sampler_state_desc_; }
    inline scm::gl::sampler_state_ptr const& get_sampler_state() const { return sampler_state_; }

  private:
    ABuffer abuffer_;

    scm::gl::frame_buffer_ptr fbo_read_;
    scm::gl::frame_buffer_ptr fbo_write_;

    scm::gl::frame_buffer_ptr fbo_read_only_color_;
    scm::gl::frame_buffer_ptr fbo_write_only_color_;

    scm::gl::sampler_state_desc sampler_state_desc_;
    scm::gl::sampler_state_ptr sampler_state_;

    scm::gl::texture_2d_ptr color_buffer_read_;
    scm::gl::texture_2d_ptr color_buffer_write_;
    scm::gl::texture_2d_ptr pbr_buffer_;
    scm::gl::texture_2d_ptr normal_buffer_;
    scm::gl::texture_2d_ptr flags_buffer_;
    scm::gl::texture_2d_ptr depth_buffer_;
};

} // namespace gua

#endif // GUA_GBUFFER_HPP
