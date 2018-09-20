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

#ifndef GUA_LAYEREDPHYSICALTEXTURE2D_HPP
#define GUA_LAYEREDPHYSICALTEXTURE2D_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/Texture.hpp>
#include <gua/math/math.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/virtual_texturing/VTInfo.hpp>

#include <scm/gl_util/data/imaging/texture_image_data.h>

// external headers
#include <string>
#include <vector>

#include <mutex>
#include <thread>
#include <memory>

namespace gua {

/**
 * A class representing a texture.
 *
 * This class allows to load texture data from a file and bind the
 * texture to an OpenGL context.
 */
class GUA_DLL LayeredPhysicalTexture2D : public Texture {
 public:

  /**
   * Constructor.
   *
   * This constructs a new texture from a given file.
   *
   * \param file             The file which contains the texture data.
   * \param state_descripton The sampler state for the loaded texture.
   */
  LayeredPhysicalTexture2D(); /*std::string const& file,
              scm::gl::sampler_state_desc const& state_descripton =
              scm::gl::sampler_state_desc(scm::gl::FILTER_ANISOTROPIC,
                                          scm::gl::WRAP_REPEAT,
                                          scm::gl::WRAP_REPEAT));*/



//Constructor 
//get path to config file and atlas
//retrieve size from backend
//VTTexture2D(gua::math::vec2ui dim, uint16_t layers, vt::VTConfig::FORMAT_TEXTURE format);

  ///@{
  /**
   * Gets the size.
   *
   * Returns the size of the Texture2D.
   */
  unsigned width() const override { return width_; }
  unsigned height() const override { return height_; }
  unsigned num_layers() const {return num_layers_; }

  void upload_to(RenderContext const& context) const override;
//  void initialize_physical_texture(RenderContext const& ctx) const;

  scm::gl::texture_2d_ptr get_physical_texture_ptr() const { return physical_texture_ptr_; }
  //static scm::gl::data_format get_tex_format();
  ///@}

 protected:
  mutable scm::gl::texture_2d_ptr physical_texture_ptr_ = nullptr;
  unsigned width_;
  unsigned height_;
  unsigned num_layers_;

 private:
  std::string file_config_;
  std::string file_atlas_;
};  

}
#endif // GUA_LAYEREDPHYSICALTEXTURE2D_HPP