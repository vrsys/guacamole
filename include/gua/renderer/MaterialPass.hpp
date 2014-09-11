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

#ifndef GUA_MATERIAL_PASS_HPP
#define GUA_MATERIAL_PASS_HPP

#include <gua/renderer/Uniform.hpp>

#include <iostream>
#include <string>
#include <memory>
#include <unordered_map>

namespace gua {

class MaterialPass {
 public:

  MaterialPass(std::string const& name = "");

  MaterialPass& load_from_file(std::string const& file_name);
  MaterialPass& load_from_json(std::string const& json_string);

  MaterialPass& set_name(std::string const& name);
  std::string const& get_name() const;

  MaterialPass& set_source(std::string const& source);
  std::string const& get_source() const;

  template <typename T>
  MaterialPass& set_uniform(std::string const& name, T const& value) {
    auto uniform(uniforms_.find(name));

    if (uniform == uniforms_.end()) {
      uniforms_[name] = std::make_shared<UniformValue<T>>(value);
    } else {
      uniform->second->set_value(value);
    }

    return *this;
  }

  std::unordered_map<std::string, std::shared_ptr<UniformValueBase>> const&
  get_uniforms() const;

 private:
  std::string name_;
  std::string source_;
  std::unordered_map<std::string, std::shared_ptr<UniformValueBase>> uniforms_;
};

}

#endif  // GUA_MATERIAL_PASS_HPP
