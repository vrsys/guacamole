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

#include <gua/renderer/MaterialPass.hpp>

#include <list>

namespace gua {

class MaterialShaderDescription {
 public:

  void load_from_file(std::string const& file_name);

  MaterialShaderDescription& add_vertex_pass(MaterialPass const& pass);

  MaterialShaderDescription& add_fragment_pass(MaterialPass const& pass);

  std::list<MaterialPass> const& get_vertex_passes() const;

  std::list<MaterialPass> const& get_fragment_passes() const;

 private:
  std::list<MaterialPass> vertex_passes_;
  std::list<MaterialPass> fragment_passes_;


};

}

#endif  // GUA_MATERIAL_SHADER_DESCRIPTION_HPP
