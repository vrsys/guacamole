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

#include <gua/renderer/ViewDependentUniform.hpp>

#include <string>
#include <vector>

namespace gua {

class MaterialShader;
class ShaderProgram;

class GUA_DLL Material {
  public:
    Material(std::string const& shader_name = "");
    Material(Material const& copy) = default;

    std::string const& get_shader_name() const {
      return shader_name_;
    }

    MaterialShader* get_shader() const;

    Material& set_uniform(std::string const& name, ViewDependentUniform const& uniform) {
      uniforms_[name] = uniform;
      return *this;
    }

    template <typename T>
    Material& set_uniform(std::string const& name, T const& value) {
      return set_uniform(name, ViewDependentUniform(UniformValue(value)));
    }

    template <typename T>
    Material& set_uniform(std::string const& name, T const& value, int view_id) {
      auto uniform(uniforms_.find(name));

      if (uniform == uniforms_.end()) {
        set_uniform(name, value);
        set_uniform(name, value, view_id);
      } else {
        uniform->second.set(view_id, value);
      }

      return *this;
    }

    std::map<std::string, ViewDependentUniform> const& get_uniforms() const;

    void apply_uniforms(RenderContext const& ctx, ShaderProgram* shader, int view) const;

  private:
    friend class MaterialShader;

    std::string shader_name_;
    mutable MaterialShader* shader_cache_;
    std::map<std::string, ViewDependentUniform> uniforms_;

};

}

#endif  // GUA_MATERIAL_HPP
