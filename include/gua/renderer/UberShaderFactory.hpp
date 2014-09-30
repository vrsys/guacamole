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

#ifndef GUA_UBER_SHADER_FACTORY_HPP
#define GUA_UBER_SHADER_FACTORY_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/UberShader.hpp>

// external headers
#include <memory>

namespace gua {

/**
 *
 */
class GUA_DLL UberShaderFactory {
 public:

  /**
   *
   */
  UberShaderFactory(ShadingModel::StageID stage,
                    std::set<std::string> const& material_names,
                    UniformMapping const& uniform_mapping = UniformMapping());

  void add_inputs_to_main_functions(
      std::vector<LayerMapping const*> const& inputs,
      ShadingModel::StageID first_input_stage);

  UniformMapping const& get_uniform_mapping() const;
  LayerMapping const& get_output_mapping() const;

  std::unordered_map<std::string, std::string> const& get_main_functions() const;
  std::unordered_map<int, std::string> const& get_main_calls() const;

  std::vector<std::string> const& get_custom_functions() const;
  std::vector<std::string> const& get_custom_function_declares() const;

 private:

  void load_main_functions(std::shared_ptr<ShadingModel> const& model);
  void load_custom_functions(std::shared_ptr<ShadingModel> const& model);

  ShadingModel::StageID stage_;

  UniformMapping uniform_mapping_;
  LayerMapping output_mapping_;

  //       shading model, code
  std::unordered_map<std::string, std::string> main_functions_;

  //       material id, source code
  std::unordered_map<int, std::string> main_calls_;

  std::vector<std::string> custom_functions_;
  std::vector<std::string> custom_function_declares_;
};

}

#endif  // GUA_UBER_SHADER_FACTORY_HPP
