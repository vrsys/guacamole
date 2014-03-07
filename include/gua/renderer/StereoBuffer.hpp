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

#ifndef GUA_STEREO_BUFFER_HPP
#define GUA_STEREO_BUFFER_HPP

// guacamole headers
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>

namespace gua {

/**
 *
 */
class StereoBuffer {
 public:
  /**
   *
   */
  StereoBuffer(
      RenderContext const& ctx,
      Pipeline::Configuration const& config,
      std::vector<std::pair<BufferComponent,
                            scm::gl::sampler_state_desc> > const& layers);

  /**
   *
   */
  ~StereoBuffer();

  void remove_buffers(RenderContext const& ctx);

  /**
   *
   */
  void clear(RenderContext const& ctx);

  /**
   *
   */
  std::vector<GBuffer*> get_eye_buffers() const;

 private:
  std::vector<GBuffer*> eye_buffers_;

  unsigned width_, height_;
};

}

#endif  // GUA_STEREO_BUFFER_HPP
