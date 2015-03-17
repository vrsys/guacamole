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

namespace gua {

class GUA_DLL GBuffer : public RenderTarget {
 public:

  GBuffer(RenderContext const& ctx, math::vec2ui const& resolution);

  void clear(RenderContext const& context) override;
  void clear_color(RenderContext const& context);
  
  void bind(RenderContext const& context, bool write_depth) override;
  void unbind(RenderContext const& context) override;

  void toggle_ping_pong();

  void allocate_a_buffer(RenderContext& ctx, size_t buffer_size);
  void remove_buffers(RenderContext const& ctx) override;

  std::shared_ptr<Texture2D> const& get_color_buffer()  const;
  std::shared_ptr<Texture2D> const& get_pbr_buffer()    const;
  std::shared_ptr<Texture2D> const& get_normal_buffer() const;
  std::shared_ptr<Texture2D> const& get_flags_buffer()  const;
  std::shared_ptr<Texture2D> const& get_depth_buffer()  const override;

 private:
  ABuffer abuffer_;

  scm::gl::frame_buffer_ptr fbo_read_;
  scm::gl::frame_buffer_ptr fbo_write_;

  scm::gl::frame_buffer_ptr fbo_read_only_color_;
  scm::gl::frame_buffer_ptr fbo_write_only_color_;

  std::shared_ptr<Texture2D> color_buffer_read_;
  std::shared_ptr<Texture2D> color_buffer_write_;
  std::shared_ptr<Texture2D> pbr_buffer_;
  std::shared_ptr<Texture2D> normal_buffer_;
  std::shared_ptr<Texture2D> flags_buffer_;
  std::shared_ptr<Texture2D> depth_buffer_;
};

}

#endif  // GUA_GBUFFER_HPP
