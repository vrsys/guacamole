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

#ifndef GUA_MATERIAL_HPP
#define GUA_MATERIAL_HPP

// guacamole headers
#include <gua/renderer/MaterialDescription.hpp>
#include <gua/renderer/Uniform.hpp>
#include <gua/platform.hpp>

// external headers
#include <memory>

namespace gua {

struct RenderContext;
class TextFile;

/**
 * Stores information on a Material.
 *
 * Materials are defined by a fragment and a vertex shader. Additionally
 * uniforms of this shaders may be set to specific values.
 */
class Material {
 public:

  /**
   * Default constructor.
   *
   * Creates a new (invalid) material. It won't do anything until being
   * initialized with the non-default constructor.
   */
  Material();

  /**
   * Default constructor.
   *
   * Creates a new (invalid) material. It won't do anything until being
   * initialized with the non-default constructor.
   */
  Material(std::string const& name);

  /**
   * Constructor from a material description.
   *
   * Creates a new Material from a given material description.
   *
   * \param file_name        The file used to describe this material.
   */
  Material(std::string const& name, MaterialDescription const& description);

  inline MaterialDescription const& get_description() const {
    return description_;
  }

  inline std::string const& get_name() const { return name_; }

  inline unsigned get_id() const { return id_; }

  inline std::unordered_map<std::string,
                            std::unique_ptr<UniformValueBase> > const&
  get_uniform_values() const {
    return uniform_values_;
  }

  void reload();

  template <typename T>
  void set_uniform(std::string const& name, T const& value) {

    auto uniform(uniform_values_.find(name));

    if (uniform == uniform_values_.end()) {
      Logger::LOG_WARNING << "Failed to set uniform: Material " << name_ << " has no uniform called " << name << "!" << std::endl;
      return;
    }

    uniform->second->set_value(value);
  }

 private:
  void load_description();

  static unsigned global_id_count_;

  std::string name_;
  unsigned id_;

  std::unordered_map<std::string, std::unique_ptr<UniformValueBase> >
      uniform_values_;
  MaterialDescription description_;

  //        scm::gl::blend_state_desc blend_state_desc_;
  //        scm::gl::rasterizer_state_desc rasterizer_state_desc_;
  //        scm::gl::depth_stencil_state_desc depth_stencil_state_desc_;
  //
  //        mutable scm::gl::blend_state_ptr blend_state_;
  //        mutable scm::gl::rasterizer_state_ptr rasterizer_state_;
  //        mutable scm::gl::depth_stencil_state_ptr depth_stencil_state_;
};

}

#endif  // GUA_MATERIAL_HPP
