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

#ifndef GUA_SHADOW_MAP_BUFFER_HPP
#define GUA_SHADOW_MAP_BUFFER_HPP

// guacamole headers
#include <gua/renderer/FrameBufferObject.hpp>
#include <gua/renderer/enums.hpp>

namespace gua {

class ShadowMapBuffer {
 public:

  ShadowMapBuffer(RenderContext const& ctx, math::vec2ui const& resolution);

  void clear(RenderContext const& context);
  void set_viewport(RenderContext const& context);

  void bind(RenderContext const& context);
  void unbind(RenderContext const& context);

  void remove_buffers(RenderContext const& ctx);

  std::shared_ptr<Texture2D> const& get_depth_buffer()  const;

  unsigned get_width()  const { return width_; }
  unsigned get_height() const { return height_; }

 private:
  std::shared_ptr<FrameBufferObject> fbo_;
  std::shared_ptr<Texture2D> depth_buffer_;

  unsigned width_, height_;
};

}

#endif  // GUA_SHADOW_MAP_BUFFER_HPP
