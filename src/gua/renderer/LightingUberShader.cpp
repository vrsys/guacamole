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

// class header
#include <gua/renderer/LightingUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/databases.hpp>
#include <gua/memory.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <sstream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void LightingUberShader::
create(std::set<std::string> const& material_names,
       std::vector<LayerMapping const*> const& inputs) {

  fshader_factory_ = gua::make_unique<UberShaderFactory>(
    ShadingModel::LIGHTING_STAGE, material_names
    );

  fshader_factory_->add_inputs_to_main_functions(inputs, ShadingModel::GBUFFER_FRAGMENT_STAGE);

  UberShader::set_uniform_mapping(fshader_factory_->get_uniform_mapping());
  UberShader::set_output_mapping(fshader_factory_->get_output_mapping());

  // VERTEX SHADER -------------------------------------------------------------
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_lighting_lighting_vert)
  );

  // FRAGMENT SHADER -----------------------------------------------------------
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_lighting_lighting_frag)
  );

  // input from gbuffer
  std::stringstream s;
  for (unsigned i(0); i<inputs.size(); ++i)
    s << inputs[i]->get_gbuffer_input_definition(ShadingModel::StageID(i+1));

  string_utils::replace(fragment_shader, "@input_definition", s.str());

  // material specific uniforms
  string_utils::replace(fragment_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // outputs
  string_utils::replace(fragment_shader, "@output_definition",
    get_gbuffer_mapping()->get_gbuffer_output_definition(false, false));

  // print material specific methods
  string_utils::replace(fragment_shader, "@material_methods",
    UberShader::print_material_methods(*fshader_factory_));

  // print main switch(es)
  string_utils::replace(fragment_shader, "@material_switch",
    UberShader::print_material_switch(*fshader_factory_));

  auto lighting_program = std::make_shared<ShaderProgram>();
  lighting_program->create_from_sources(vertex_shader, fragment_shader);
  add_program(lighting_program);
}

}
