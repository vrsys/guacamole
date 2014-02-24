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
#include <gua/renderer/GBufferMeshUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/databases.hpp>
#include <gua/utils/logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void GBufferMeshUberShader::create(std::set<std::string> const& material_names) {

  UberShaderFactory vshader_factory(
    ShadingModel::GBUFFER_VERTEX_STAGE, material_names
  );

  UberShaderFactory fshader_factory(
    ShadingModel::GBUFFER_FRAGMENT_STAGE, material_names,
    vshader_factory.get_uniform_mapping()
  );

  LayerMapping vshader_output_mapping = vshader_factory.get_output_mapping();

  fshader_factory.add_inputs_to_main_functions(
    {&vshader_output_mapping}, ShadingModel::GBUFFER_VERTEX_STAGE
  );

  UberShader::set_uniform_mapping(fshader_factory.get_uniform_mapping());
  UberShader::set_output_mapping(fshader_factory.get_output_mapping());

  // VERTEX SHADER -------------------------------------------------------------
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_mesh_mesh_vert)
  );

  // material specific uniforms
  string_utils::replace(vertex_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // output
  string_utils::replace(vertex_shader, "@output_definition",
    vshader_output_mapping.get_gbuffer_output_definition(false, true));

  // print material specific methods
  string_utils::replace(vertex_shader, "@material_methods",
    UberShader::print_material_methods(vshader_factory));

  // print main switch(es)
  string_utils::replace(vertex_shader, "@material_switch",
    UberShader::print_material_switch(vshader_factory));

  // FRAGMENT SHADER -----------------------------------------------------------
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_mesh_mesh_frag)
  );

  // input from vertex shader
  string_utils::replace(fragment_shader, "@input_definition",
    vshader_output_mapping.get_gbuffer_output_definition(true, true));

  // material specific uniforms
  string_utils::replace(fragment_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // outputs
  string_utils::replace(fragment_shader, "@output_definition",
    get_gbuffer_mapping()->get_gbuffer_output_definition(false, false));

  // print material specific methods
  string_utils::replace(fragment_shader, "@material_methods",
    UberShader::print_material_methods(fshader_factory));

  // print main switch(es)
  string_utils::replace(fragment_shader, "@material_switch",
    UberShader::print_material_switch(fshader_factory));

  create_from_sources(vertex_shader, fragment_shader);

 
}

}

