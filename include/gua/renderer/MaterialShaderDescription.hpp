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

#ifndef GUA_MATERIAL_SHADER_DESCRIPTION_HPP
#define GUA_MATERIAL_SHADER_DESCRIPTION_HPP

#include <gua/renderer/MaterialShaderMethod.hpp>

#include <list>

namespace gua {

class GUA_DLL MaterialShaderDescription {
 public:
  MaterialShaderDescription() : file_name_("") {}


  void load_from_file(std::string const& file_name);
  std::string const& get_file_name() const {
    return file_name_;
  }

  MaterialShaderDescription& add_vertex_method(MaterialShaderMethod const& method);
  MaterialShaderDescription& add_fragment_method(MaterialShaderMethod const& method);

  std::list<MaterialShaderMethod> const& get_vertex_methods() const;
  std::list<MaterialShaderMethod> const& get_fragment_methods() const;

 private:
  std::list<MaterialShaderMethod> vertex_methods_;
  std::list<MaterialShaderMethod> fragment_methods_;

  std::string file_name_;


};

}

#endif  // GUA_MATERIAL_SHADER_DESCRIPTION_HPP
