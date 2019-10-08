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

#ifndef GUA_TEXTURE_DISTANCE_HPP
#define GUA_TEXTURE_DISTANCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/math/math.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <string>
#include <vector>

#include <mutex>
#include <thread>

namespace gua
{
/**
 * A class representing a texture.
 *
 * This class allows to load texture data from a file and bind the
 * texture to an OpenGL context.
 */
class GUA_DLL TextureDistance : public Texture2D
{
  public:
    /**
     * Constructor.
     *
     * This constructs a new texture with the given parameters.
     *
     * \param width            The width of the resulting texture.
     * \param height           The height of the resulting texture.
     * \param color_format     The color format of the resulting
     *                         texture.
     * \param state_descripton The sampler state for the loaded texture.
     */
    TextureDistance(unsigned width,
                    unsigned height,
                    scm::gl::data_format color_format,
                    unsigned mipmap_layers = 1,
                    scm::gl::sampler_state_desc const& state_descripton = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_MIP_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE, scm::gl::WRAP_CLAMP_TO_EDGE));

    virtual ~TextureDistance() {}

    void download_data(RenderContext const& ctx, float near_clip, float far_clip);
    std::vector<float> const& get_data();

  protected:
  private:
    uint32_t* texture_data_;
    std::vector<float> world_depth_data_;
};

} // namespace gua
#endif // GUA_TEXTURE_DISTANCE_HPP
