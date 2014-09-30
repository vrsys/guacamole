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

#include <gua/renderer/Uniform.hpp>

#include <string>
#include <vector>

namespace gua {

class Material {
  public:
    Material(std::string const& material_name = "");

    std::string const& get_material_name() const {
      return material_name_;
    }

    Material& set_uniform(UniformValue const& uniform) {
      for (auto& val: uniforms_) {
        if (uniform.get_name() == val.get_name()) {
          val = uniform;
          return *this;
        }
      }
      uniforms_.push_back(uniform);
      return *this;
    }

    template <typename T>
    Material& set_uniform(std::string const& name, T const& value) {
      return set_uniform(UniformValue(name, value));
    }

    // void unset_uniform(std::string const& name);

    std::vector<UniformValue> const& get_uniforms() const;

  private:
    friend class MaterialShader;

    std::string material_name_;
    std::vector<UniformValue> uniforms_;

};

}

#endif  // GUA_MATERIAL_HPP
