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

#include <gua/renderer/MaterialDescription.hpp>
#include <gua/renderer/MaterialInstance.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/string_utils.hpp>

#include <typeindex>
#include <sstream>
#include <iostream>

namespace gua {

class Material {
 public:

  Material(std::string const& name, MaterialDescription const& desc);

  MaterialDescription const& get_description() const;

  std::string const&      get_name()             const;
  MaterialInstance const  get_new_instance()     const;
  MaterialInstance const& get_default_instance() const;
  MaterialInstance&       get_default_instance();

  ShaderProgram* get_shader(GeometryResource const& for_type,
                            MaterialInstance const& overwrite = MaterialInstance());

  void print_shaders() const;

 private:

  std::string compile_description(std::list<MaterialPass> const& passes,
                                  std::string const& shader_source) const;

  MaterialDescription desc_;

  std::unordered_map<std::type_index, ShaderProgram*> shaders_;

  MaterialInstance default_instance_;
};

}

#endif  // GUA_MATERIAL_HPP
