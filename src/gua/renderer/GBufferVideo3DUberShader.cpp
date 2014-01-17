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
#include <gua/renderer/GBufferVideo3DUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases.hpp>
#include <gua/utils/logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void GBufferVideo3DUberShader::create(std::set<std::string> const& material_names) {

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
  //see _final_vertex_shader(UberShaderFactory const& vshader_factory)

  // FRAGMENT SHADER -----------------------------------------------------------
  //see _final_fragment_shader(UberShaderFactory const& fshader_factory)

  //&create_from_sources(vertex_shader, fragment_shader);

  std::vector<ShaderProgramStage> shader_stages;
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          _final_vertex_shader(vshader_factory, vshader_output_mapping)));
  //shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_GEOMETRY_SHADER,        _final_geometry_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        _final_fragment_shader(fshader_factory, vshader_output_mapping)));

  // generate shader source
  set_shaders ( shader_stages );

  //save shader sources for debugging
  save_to_file(".", "final_pass");
}

std::string const GBufferVideo3DUberShader::_final_vertex_shader(UberShaderFactory const& vshader_factory, LayerMapping const& vshader_output_mapping) const
{
  
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_mesh_vert) //TODO gbuffer_video3d_video3d_vert
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
  
  return vertex_shader;
}

std::string const GBufferVideo3DUberShader::_final_geometry_shader() const
{
  std::string geometry_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_geom) //TODO gbuffer_video3d_video3d_vert
  );

  return geometry_shader;
  //TODO
}

std::string const GBufferVideo3DUberShader::_final_fragment_shader(UberShaderFactory const& fshader_factory, LayerMapping const& vshader_output_mapping) const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_mesh_frag)//TODO gbuffer_video3d_video3d_frag
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

  return fragment_shader;
}

}

