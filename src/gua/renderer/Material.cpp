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

#include <gua/renderer/Material.hpp>

#include <gua/databases/MaterialShaderDatabase.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Material::Material(std::string const& shader_name):
  shader_name_(shader_name),
  shader_cache_(nullptr)
  {}

////////////////////////////////////////////////////////////////////////////////

MaterialShader* Material::get_shader() const {
  if (!shader_cache_) {
    shader_cache_ = MaterialShaderDatabase::instance()->lookup(shader_name_).get();
  }

  return shader_cache_;
}

////////////////////////////////////////////////////////////////////////////////

std::map<std::string, ViewDependentUniform> const& Material::get_uniforms() const {
  return uniforms_;
}

}
