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
#include <gua/renderer/FrameBufferObject.hpp>
#include <gua/renderer/enums.hpp>

namespace gua {

class PipelinePass;

class GBuffer {
 public:

  GBuffer(RenderContext const& ctx, math::vec2ui const& resolution);
  virtual ~GBuffer() {}

  void clear_all(RenderContext const& context);
  void clear_color(RenderContext const& context);
  
  void set_viewport(RenderContext const& context);

  void bind(RenderContext const& context, PipelinePass* next_pass);
  void unbind(RenderContext const& context);

  void toggle_ping_pong();

  void remove_buffers(RenderContext const& ctx);

  std::shared_ptr<Texture2D> const& get_current_color_buffer()  const;
  std::shared_ptr<Texture2D> const& get_current_pbr_buffer()    const;
  std::shared_ptr<Texture2D> const& get_current_normal_buffer() const;
  std::shared_ptr<Texture2D> const& get_current_depth_buffer()  const;

  unsigned get_width()  const { return width_; }
  unsigned get_height() const { return height_; }

 private:
  std::shared_ptr<FrameBufferObject> fbo_read_;
  std::shared_ptr<FrameBufferObject> fbo_write_;
  
  std::shared_ptr<FrameBufferObject> fbo_read_only_color_;
  std::shared_ptr<FrameBufferObject> fbo_write_only_color_;

  std::shared_ptr<Texture2D> color_buffer_read_;
  std::shared_ptr<Texture2D> color_buffer_write_;
  std::shared_ptr<Texture2D> pbr_buffer_;
  std::shared_ptr<Texture2D> normal_buffer_;
  std::shared_ptr<Texture2D> depth_buffer_;

  unsigned width_, height_;
};

}

#endif  // GUA_GBUFFER_HPP
