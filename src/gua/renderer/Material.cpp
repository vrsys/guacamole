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

#include <gua/renderer/Material.hpp>

#include <gua/utils/string_utils.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
Material::Material(std::string const& name, MaterialDescription const& desc)
  : desc_(desc),
    default_instance_(name)
{
  auto v_passes = desc_.get_vertex_passes();
  auto f_passes = desc_.get_fragment_passes();

  for (auto const& pass : v_passes) {
    for (auto const& uniform : pass.get_uniforms()) {
      default_instance_.set_uniform(uniform);
    }
  }

  for (auto const& pass : f_passes) {
    for (auto const& uniform : pass.get_uniforms()) {
      default_instance_.set_uniform(uniform);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
MaterialDescription const& Material::get_description() const {
  return desc_;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& Material::get_name() const {
  return default_instance_.get_material_name();
}

////////////////////////////////////////////////////////////////////////////////
MaterialInstance const Material::get_new_instance() const {
  return MaterialInstance(default_instance_.get_material_name());
}

////////////////////////////////////////////////////////////////////////////////
MaterialInstance const& Material::get_default_instance() const {
  return default_instance_;
}

////////////////////////////////////////////////////////////////////////////////
MaterialInstance& Material::get_default_instance() {
  return default_instance_;
}

////////////////////////////////////////////////////////////////////////////////
ShaderProgram* Material::get_shader(GeometryResource const& for_type,
                                    std::string const& geometry_v_shader,
                                    std::string const& geometry_f_shader) {

  auto shader(shaders_.find(typeid(*for_type)));

  if (shader != shaders_.end()) {
    return shader->second;
  } else {

    auto v_passes = desc_.get_vertex_passes();
    auto f_passes = desc_.get_fragment_passes();

    auto new_shader = new ShaderProgram();

    // std::cout << "VERTEX SHADER " << std::endl;
    // std::cout << "################################# " << std::endl << std::endl;
    auto v_shader(compile_description(v_passes, geometry_v_shader));

    // std::cout << std::endl << "FRAGMENT SHADER " << std::endl;
    // std::cout << "################################# " << std::endl << std::endl;

    auto f_shader(compile_description(f_passes, geometry_f_shader));

    new_shader->create_from_sources(v_shader, f_shader);

    shaders_[type_id] = new_shader;

    return new_shader;
  }
}

////////////////////////////////////////////////////////////////////////////////

void Material::apply_uniforms(RenderContext const& ctx,
                              ShaderProgram* shader,
                              MaterialInstance const& overwrite) const {


  for (auto const& uniform : default_instance_.get_uniforms()) {
    shader->apply_uniform(ctx, uniform);
  }

  for (auto const& uniform : overwrite.get_uniforms()) {
    shader->apply_uniform(ctx, uniform);
  }
}

////////////////////////////////////////////////////////////////////////////////

void Material::print_shaders() const {
  // for (auto shader: shaders_) {
  //   shader.second->print_shaders();
  // }
}

////////////////////////////////////////////////////////////////////////////////
std::string Material::compile_description(std::list<MaterialPass> const& passes,
                                          std::string const& shader_source) const {
  std::string source(shader_source);
  std::stringstream uniforms;

  std::unordered_map<std::string, UniformValue const*> pass_uniforms;

  // collect uniforms from all passes
  for (auto const& pass: passes) {
    for (auto const& uniform: pass.get_uniforms()) {
      pass_uniforms[uniform.get_name()] = &uniform;
    }
  }

  for (auto const& uniform: pass_uniforms) {
    uniforms << "uniform "
           << uniform.second->get_glsl_type() << " "
           << uniform.first << ";"
           << std::endl;
  }

  // insert uniforms
  gua::string_utils::replace(source, "@material_uniforms", uniforms.str());


  std::stringstream method_declarations;
  std::stringstream method_calls;

  // pass sources ----------------------------------------------------------
  for (auto& pass: passes) {
    method_declarations << pass.get_source() << std::endl;
    method_calls << pass.get_name() << "()" << std::endl;
  }
  gua::string_utils::replace(source, "@material_method_declarations", method_declarations.str());
  gua::string_utils::replace(source, "@material_method_calls", method_calls.str());

  // indent and return code ------------------------------------------------
  return string_utils::format_code(source);
}

}
