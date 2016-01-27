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

class Pipeline;

class GUA_DLL GBuffer : public RenderTarget {
 public:

  GBuffer(RenderContext const& ctx, math::vec2ui const& resolution);

  void clear(RenderContext const& context, float depth = 1.f, unsigned stencil = 0) override;
  void clear_all(RenderContext const& context, float depth = 1.f, unsigned stencil = 0);
  void clear_abuffer(RenderContext const& context);
  
  void bind(RenderContext const& context, bool write_all_layers, bool do_clear, bool do_swap) override;
  void unbind(RenderContext const& context) override;

  ABuffer& get_abuffer();

  void remove_buffers(RenderContext const& ctx) override;

  std::shared_ptr<Texture2D> const& get_color_buffer()  const;
  std::shared_ptr<Texture2D> const& get_pbr_buffer()    const;
  std::shared_ptr<Texture2D> const& get_normal_buffer() const;
  std::shared_ptr<Texture2D> const& get_depth_buffer()  const override;

  std::shared_ptr<Texture2D> const& get_color_buffer_write()  const;
  std::shared_ptr<Texture2D> const& get_pbr_buffer_write()    const;
  std::shared_ptr<Texture2D> const& get_normal_buffer_write() const;
  std::shared_ptr<Texture2D> const& get_depth_buffer_write()  const;

  inline scm::gl::frame_buffer_ptr get_fbo_read() const { return fbo_read_; }

 private:
  ABuffer abuffer_;

  scm::gl::frame_buffer_ptr fbo_read_;
  scm::gl::frame_buffer_ptr fbo_write_;

  scm::gl::frame_buffer_ptr fbo_read_only_color_;
  scm::gl::frame_buffer_ptr fbo_write_only_color_;

  std::shared_ptr<Texture2D> color_buffer_r_;
  std::shared_ptr<Texture2D> color_buffer_w_;
  std::shared_ptr<Texture2D> pbr_buffer_r_;
  std::shared_ptr<Texture2D> pbr_buffer_w_;
  std::shared_ptr<Texture2D> normal_buffer_r_;
  std::shared_ptr<Texture2D> normal_buffer_w_;
  std::shared_ptr<Texture2D> depth_buffer_r_;
  std::shared_ptr<Texture2D> depth_buffer_w_;
};

}

#endif  // GUA_GBUFFER_HPP
