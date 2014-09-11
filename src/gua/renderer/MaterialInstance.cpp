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

#include <gua/renderer/MaterialInstance.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
MaterialInstance::MaterialInstance(std::string const& material_name):
  material_name_(material_name)
  {}

////////////////////////////////////////////////////////////////////////////////
MaterialInstance::MaterialInstance(MaterialInstance const& to_copy) {
  material_name_ = to_copy.material_name_;

  for (auto const& uniform : to_copy.uniforms_) {
    uniforms_.insert(
      std::make_pair(uniform.first, uniform.second->get_copy()));
  }
}

////////////////////////////////////////////////////////////////////////////////
MaterialInstance::~MaterialInstance() {
  for (auto const& uniform : uniforms_) {
    delete uniform.second;
  }
}

////////////////////////////////////////////////////////////////////////////////
void MaterialInstance::unset_uniform(std::string const& name) {
  auto pos(uniforms_.find(name));

  if (pos != uniforms_.end()) {
    delete pos->second;
    uniforms_.erase(pos);
  }
}

////////////////////////////////////////////////////////////////////////////////
std::unordered_map<std::string, UniformValueBase*> const&
                            MaterialInstance::get_uniforms() const {
  return uniforms_;
}

////////////////////////////////////////////////////////////////////////////////
void MaterialInstance::merge(MaterialInstance const& to_merge) {
  if (material_name_ == to_merge.material_name_) {
    uniforms_.insert(to_merge.uniforms_.begin(), to_merge.uniforms_.end());
  } else {
    //WARNING
  }
}

////////////////////////////////////////////////////////////////////////////////
void MaterialInstance::operator= (MaterialInstance const& rhs) {
  material_name_ = rhs.material_name_;

  for (auto const& uniform : rhs.uniforms_) {
    uniforms_.insert(
      std::make_pair(uniform.first, uniform.second->get_copy()));
  }
}

}
