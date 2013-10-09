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

#ifndef GUA_UBER_SHADER_HPP
#define GUA_UBER_SHADER_HPP

// guacamole headers
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/ShadingModel.hpp>
#include <gua/renderer/UniformMapping.hpp>
#include <gua/renderer/LayerMapping.hpp>
#include <gua/renderer/enums.hpp>

#define CASES_PER_UBERSHADER_SWITCH 30

namespace gua {

class UberShaderFactory;

/**
 *
 */
class UberShader : public ShaderProgram {
 public:
  /**
   * Default constructor.
   *
   * Creates a new (invalid) shader program.
   */
  UberShader();

  /**
   *
   */
  void set_material_uniforms(std::set<std::string> const& materials,
                             ShadingModel::StageID stage,
                             RenderContext const& context);

  /**
   *
   */
  virtual LayerMapping const* get_gbuffer_mapping() const;

  /**
   *
   */
  virtual UniformMapping const* get_uniform_mapping() const;

 protected:
  void set_uniform_mapping(UniformMapping const& mapping);
  void set_output_mapping(LayerMapping const& mapping);

  std::string const print_material_switch(UberShaderFactory const& factory) const;
  std::string const print_material_methods(UberShaderFactory const& factory) const;

 private:
  UniformMapping uniform_mapping_;
  LayerMapping output_mapping_;
};

}

#endif  // GUA_UBER_SHADER_HPP
