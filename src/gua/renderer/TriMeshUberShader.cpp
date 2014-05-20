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
#include <gua/renderer/TriMeshUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////

  void TriMeshUberShader::create(std::set<std::string> const& material_names)
  {
    // create ubershader factories for given material set
    UberShader::create(material_names);

    // VERTEX SHADER -------------------------------------------------------------
    std::string vertex_shader(
      Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_mesh_mesh_vert)
      );

    // material specific uniforms
    string_utils::replace(vertex_shader, "@uniform_definition",
      get_uniform_mapping()->get_uniform_definition());

    // output
    string_utils::replace(vertex_shader, "@output_definition",
      vshader_factory_->get_output_mapping().get_gbuffer_output_definition(false, true));

    // print material specific methods
    string_utils::replace(vertex_shader, "@material_methods",
      UberShader::print_material_methods(*vshader_factory_));

    // print main switch(es)
    string_utils::replace(vertex_shader, "@material_switch",
      UberShader::print_material_switch(*vshader_factory_));

    // FRAGMENT SHADER -----------------------------------------------------------
    std::string fragment_shader(
      Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_mesh_mesh_frag)
      );

    // input from vertex shader
    string_utils::replace(fragment_shader, "@input_definition",
      vshader_factory_->get_output_mapping().get_gbuffer_output_definition(true, true));

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

    auto program = std::make_shared<ShaderProgram>();
    program->create_from_sources(vertex_shader, fragment_shader);
    add_program(program);
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ GeometryUberShader::stage_mask const TriMeshUberShader::get_stage_mask() const
  {
    return GeometryUberShader::DRAW_STAGE;
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void  TriMeshUberShader::preframe(RenderContext const& context) const
  {
    throw std::runtime_error("TriMeshUberShader::preframe(): not implemented");
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void  TriMeshUberShader::predraw(RenderContext const& ctx,
                                               std::string const& filename,
                                               std::string const& material_name,
                                               scm::math::mat4 const& model_matrix,
                                               scm::math::mat4 const& normal_matrix,
                                               Frustum const& /*frustum*/,
                                               std::size_t /*viewid*/) const
  {
    throw std::runtime_error("TriMeshUberShader::predraw(): not implemented");
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void TriMeshUberShader::draw(RenderContext const& ctx,
                                           std::string const& filename,
                                           std::string const& material_name,
                                           scm::math::mat4 const& model_matrix,
                                           scm::math::mat4 const& normal_matrix,
                                           Frustum const& /*frustum*/,
                                           std::size_t /*viewid*/) const
  {
    auto geometry = std::static_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup(filename));
    auto material = MaterialDatabase::instance()->lookup(material_name);

    get_program()->use(ctx);
    {
      if (material && geometry)
      {
        set_uniform(ctx, material->get_id(), "gua_material_id");
        set_uniform(ctx, model_matrix, "gua_model_matrix");
        set_uniform(ctx, normal_matrix, "gua_normal_matrix");

        geometry->draw(ctx);
      }
    }
    get_program()->unuse(ctx);
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void TriMeshUberShader::postdraw(RenderContext const& ctx,
    std::string const& filename,
    std::string const& material_name,
    scm::math::mat4 const& model_matrix,
    scm::math::mat4 const& normal_matrix,
    Frustum const& /*frustum*/,
    std::size_t /*viewid*/) const
  {
    throw std::runtime_error("TriMeshUberShader::postdraw(): not implemented");
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void TriMeshUberShader::postframe(RenderContext const& context) const
  {
    throw std::runtime_error("TriMeshUberShader::postframe(): not implemented");
  }

}

