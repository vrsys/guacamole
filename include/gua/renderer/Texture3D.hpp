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

#ifndef GUA_TEXTURE3D_HPP
#define GUA_TEXTURE3D_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/Texture.hpp>
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
class GUA_DLL Texture3D : public Texture
{
  public:
    /**
     * Constructor.
     *
     * This constructs a new texture with the given parameters.
     *
     * \param width            The width of the resulting 3D texture.
     * \param height           The height of the resulting 3D texture.
     * \param depth            The depth of the resulting 3D texture.
     * \param color_format     The color format of the resulting 3D
     *                         texture.
     * \param state_descripton The sampler state for the loaded 3D texture.
     */
    Texture3D(unsigned width,
              unsigned height,
              unsigned depth,
              scm::gl::data_format color_format,
              scm::gl::data_format internal_format,
              std::vector<void*> const& data,
              unsigned mipmap_layers = 1,
              scm::gl::sampler_state_desc const& state_descripton = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE, scm::gl::WRAP_CLAMP_TO_EDGE));

    /**
     * Constructor.
     *
     * This constructs a new texture with the given parameters.
     *
     * \param width            The width of the resulting texture.
     * \param height           The height of the resulting texture.
     * \param depth            The depth of the resulting 3D texture.
     * \param color_format     The color format of the resulting
     *                         texture.
     * \param state_descripton The sampler state for the loaded texture.
     */
    Texture3D(unsigned width,
              unsigned height,
              unsigned depth,
              scm::gl::data_format color_format = scm::gl::FORMAT_RGB_32F,
              unsigned mipmap_layers = 1,
              scm::gl::sampler_state_desc const& state_descripton = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_MIP_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE, scm::gl::WRAP_CLAMP_TO_EDGE));

    /**
     * Constructor.
     *
     * This constructs a new texture from a given file.
     *
     * \param file             The file which contains the texture data.
     * \param state_descripton The sampler state for the loaded texture.
     */
    Texture3D(std::string const& file,
              bool generate_mipmaps = false,
              scm::gl::sampler_state_desc const& state_descripton = scm::gl::sampler_state_desc(scm::gl::FILTER_ANISOTROPIC, scm::gl::WRAP_REPEAT, scm::gl::WRAP_REPEAT));

    ///@{
    /**
     * Gets the size.
     *
     * Returns the size of the Texture3D.
     */
    inline unsigned width() const override { return width_; }
    inline unsigned height() const override { return height_; }
    inline unsigned depth() const { return depth_; }

    void upload_to(RenderContext const& context) const override;
    ///@}

  protected:
    mutable unsigned width_;
    mutable unsigned height_;
    mutable unsigned depth_;
    mutable std::vector<void*> data_;

  private:
};

} // namespace gua
#endif // GUA_TEXTURE3D_HPP
