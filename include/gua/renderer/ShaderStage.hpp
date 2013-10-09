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

#ifndef GUA_SHADER_STAGE_HPP
#define GUA_SHADER_STAGE_HPP

// guacamole headers
#include <gua/renderer/enums.hpp>

// external headers
#include <string>
#include <set>
#include <unordered_map>
#include <jsoncpp/json/json.h>

namespace gua {
/**
 * Stores information on a ShaderStage.
 *
 *
 */
class ShaderStage {
 public:

  /**
   * Default constructor.
   *
   */
  ShaderStage();

  /**
   * Constructor from a ShaderStage description.
   *
   */
  ShaderStage(Json::Value const& value);

  std::unordered_map<std::string, UniformType>& get_uniforms();
  std::unordered_map<std::string, BufferComponent>& get_outputs();
  std::string& get_functions();
  std::string& get_body();

  Json::Value const to_json_string() const;

 private:
  void construct_from_json_string(Json::Value const& value);
  Json::Value const uniforms_to_json(
      std::unordered_map<std::string, UniformType> const& uniforms) const;
  Json::Value const set_to_json(std::set<std::string> const& set) const;

  Json::Value const outputs_to_json(
      std::unordered_map<std::string, BufferComponent> const& outputs) const;

  std::unordered_map<std::string, UniformType> 
    json_to_uniform(Json::Value const& value);
  std::set<std::string> json_to_set(Json::Value const& value);

  std::unordered_map<std::string, BufferComponent> 
    json_to_outputs(Json::Value const& value);

  std::unordered_map<std::string, UniformType>      uniforms_;
  std::unordered_map<std::string, BufferComponent>  outputs_;
  std::string                                       functions_;
  std::string                                       body_;
};

}

#endif  // GUA_SHADER_STAGE_HPP
