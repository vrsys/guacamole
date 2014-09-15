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

#ifndef GUA_MATERIAL_INSTANCE_HPP
#define GUA_MATERIAL_INSTANCE_HPP

#include <gua/renderer/Uniform.hpp>

#include <string>
#include <unordered_map>

namespace gua {

class MaterialInstance {
  public:
    MaterialInstance(std::string const& material_name = "");

    MaterialInstance(MaterialInstance const& to_copy);
    ~MaterialInstance();

    std::string const& get_material_name() const {
      return material_name_;
    }

    template <typename T>
    MaterialInstance& set_uniform(std::string const& name, T const& value) {
      auto uniform(uniforms_.find(name));

      if (uniform == uniforms_.end()) {
        uniforms_[name] = new UniformValue<T>(value);
      } else {
        uniform->second->set_value(value);
      }

      return *this;
    }

    void unset_uniform(std::string const& name);

    std::unordered_map<std::string, UniformValueBase*> const&
                                                           get_uniforms() const;

    void merge(MaterialInstance const& to_merge);

    void operator= (MaterialInstance const& rhs);

  private:
    friend class Material;

    std::string material_name_;
    std::unordered_map<std::string, UniformValueBase*> uniforms_;

};

}

#endif  // GUA_MATERIAL_INSTANCE_HPP
