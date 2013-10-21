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

#ifndef GUA_NEW_MATERIAL_HPP
#define GUA_NEW_MATERIAL_HPP

// guacamole headers
#include <gua/renderer/ShadingModel.hpp>

#include <unordered_map>

namespace gua {

struct RenderContext;
class TextFile;

/**
 * Stores information on a MaterialDescription.
 *
 */
class MaterialDescription {
 public:

  /**
   * Default constructor.
   *
   * Creates a new (invalid) material. It won't do anything until being
   * initialized with the non-default constructor.
   */
  MaterialDescription();

  /**
   * Constructor from a material description.
   *
   * Creates a new MaterialDescription from a given material description.
   *
   * \param file_name        The file used to describe this material.
   */
  MaterialDescription(std::string const& file_name);

  /**
   * Constructor from a buffer.
   *
   * Creates a new MaterialDescription from a given buffer.
   *
   * \param buffer        A buffer containing a material description.
   * \param buffer_size   The size of the given buffer.
   */
  MaterialDescription(const char* buffer, unsigned buffer_size);

  /**
   * Destructor.
   *
   * Deletes the MaterialDescription and frees all associated data.
   */
  ~MaterialDescription();

  void reload();

  inline std::string& get_shading_model() { return shading_model_; }
  inline std::string const& get_shading_model() const {
    return shading_model_;
  }

  inline std::unordered_map<std::string, std::string>& get_uniforms() {
      return uniforms_;
  }

  inline std::unordered_map<std::string, std::string> const& get_uniforms() const {
      return uniforms_;
  }

  void save_to_file(std::string const& file_name) const;

 private:
  void construct_from_file(TextFile const& file);
  void construct_from_buffer(const char* buffer, unsigned buffer_size);

  std::string file_name_;

  std::string shading_model_;
  std::unordered_map<std::string, std::string> uniforms_;
};

}

#endif  // GUA_NEW_MATERIAL_HPP
