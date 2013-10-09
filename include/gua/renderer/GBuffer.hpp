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

/**
 *
 */
class GBuffer : public FrameBufferObject {
 public:

  /**
   *
   */
  GBuffer(std::vector<std::pair<BufferComponent,
                                scm::gl::sampler_state_desc> > const& layers,
          unsigned width,
          unsigned height,
          unsigned mipmap_layers = 1);

  void remove_buffers(RenderContext const& ctx);

  /**
   *
   */
  void create(RenderContext const& ctx);

  /**
   *
   */
  void create_UGLY(RenderContext const& ctx);

  /**
   *
   */
  std::vector<std::shared_ptr<Texture> > const& get_color_buffers(
      BufferComponentType type) const;
  std::shared_ptr<Texture> const& get_depth_buffer() const;

 private:
  std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> >
      layer_types_;
  unsigned width_, height_, mipmap_layers_;

  std::map<BufferComponentType, std::vector<std::shared_ptr<Texture> > >
      color_buffers_;
  std::shared_ptr<Texture> depth_buffer_;
};

}

#endif  // GUA_GBUFFER_HPP
