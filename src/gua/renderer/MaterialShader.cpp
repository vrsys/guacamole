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

#include <gua/renderer/MaterialShader.hpp>

#include <gua/utils/string_utils.hpp>

#define USE_UBO 0

namespace gua {

////////////////////////////////////////////////////////////////////////////////
MaterialShader::MaterialShader(std::string const& name, MaterialShaderDescription const& desc)
  : desc_(desc),
    default_material_(name),
    max_object_count_(0)
{
  auto v_methods = desc_.get_vertex_methods();
  auto f_methods = desc_.get_fragment_methods();

  for (auto const& method : v_methods) {
    for (auto const& uniform : method.get_uniforms()) {
      default_material_.set_uniform(uniform.first, uniform.second);
    }
  }

  for (auto const& method : f_methods) {
    for (auto const& uniform : method.get_uniforms()) {
      default_material_.set_uniform(uniform.first, uniform.second);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
MaterialShaderDescription const& MaterialShader::get_description() const {
  return desc_;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& MaterialShader::get_name() const {
  return default_material_.get_shader_name();
}

////////////////////////////////////////////////////////////////////////////////
Material const MaterialShader::get_new_material() const {
  return Material(default_material_.get_shader_name());
}

////////////////////////////////////////////////////////////////////////////////
Material const& MaterialShader::get_default_material() const {
  return default_material_;
}

////////////////////////////////////////////////////////////////////////////////
Material& MaterialShader::get_default_material() {
  return default_material_;
}

////////////////////////////////////////////////////////////////////////////////
ShaderProgram* MaterialShader::get_shader(RenderContext const& ctx,
                                          GeometryResource const& for_type,
                                          std::string const& geometry_v_shader,
                                          std::string const& geometry_f_shader) {

  std::type_index type_id(typeid(for_type));
  auto shader(shaders_.find(type_id));

  if (shader != shaders_.end()) {
    return shader->second;
  } else {

    auto v_methods = desc_.get_vertex_methods();
    auto f_methods = desc_.get_fragment_methods();

    auto new_shader = new ShaderProgram();

    auto v_shader(compile_description(ctx, v_methods, geometry_v_shader));
    auto f_shader(compile_description(ctx, f_methods, geometry_f_shader));

    // std::cout << "VERTEX" << std::endl;
    // std::cout << v_shader << std::endl << std::endl;
    // std::cout << "FRAGMENT" << std::endl;
    // std::cout << f_shader << std::endl;

    new_shader->create_from_sources(v_shader, f_shader);

    shaders_[type_id] = new_shader;
    return new_shader;
  }
}

////////////////////////////////////////////////////////////////////////////////

void MaterialShader::apply_uniforms(RenderContext const& ctx,
                                    ShaderProgram* shader,
                                    Material const& overwrite) const {


  // for (auto const& uniform : default_material_.get_uniforms()) {
  //   shader->apply_uniform(ctx, uniform);
  // }

  for (auto const& uniform : overwrite.get_uniforms()) {
    shader->apply_uniform(ctx, uniform.first, uniform.second.get());
  }
}

////////////////////////////////////////////////////////////////////////////////

void MaterialShader::print_shaders() const {
  // for (auto shader: shaders_) {
  //   shader.second->print_shaders();
  // }
}

////////////////////////////////////////////////////////////////////////////////
std::string MaterialShader::compile_description(RenderContext const& ctx,
                                                std::list<MaterialShaderMethod> const& methods,
                                                std::string const& shader_source) const {
  std::string source(shader_source);
  std::stringstream sstr;

  #if USE_UBO

  unsigned vec4_count(0);

  sstr << "struct GuaObjectDataStruct {" << std::endl;
  sstr << "  mat4 gua_model_matrix;" << std::endl;
  vec4_count += 4;
  sstr << "  mat4 gua_normal_matrix;" << std::endl;
  vec4_count += 4;

  for (auto const& uniform: get_default_material().get_uniforms()) {
    sstr << uniform.second.get().get_glsl_type() << " "
           << uniform.first << ";" << std::endl;
    ++vec4_count;
  }

  max_object_count_ = ctx.render_device->capabilities()._max_uniform_block_size / (vec4_count * 16);

  sstr << "};" << std::endl;
  sstr << std::endl;
  sstr << "layout (std140, binding=1) uniform GuaObjectData {" << std::endl;
  sstr << "  GuaObjectDataStruct gua_object_data[" << max_object_count_ << "];" << std::endl;
  sstr << "};" << std::endl;
  sstr << std::endl;
  sstr << "uniform int gua_draw_index;" << std::endl;
  sstr << std::endl;
  sstr << "mat4 gua_model_matrix;" << std::endl;
  sstr << "mat4 gua_normal_matrix;" << std::endl;

  for (auto const& uniform: get_default_material().get_uniforms()) {
    sstr << uniform.second.get().get_glsl_type() << " " << uniform.first + ";" << std::endl;
  }

  // insert uniforms
  gua::string_utils::replace(source, "@material_uniforms", sstr.str());
  sstr.str("");

  // global variable assignment ------------------------------------------------
  sstr << "gua_model_matrix = gua_object_data[gua_draw_index].gua_model_matrix;" << std::endl;
  sstr << "gua_normal_matrix = gua_object_data[gua_draw_index].gua_normal_matrix;" << std::endl;
  for (auto const& uniform: get_default_material().get_uniforms()) {
    sstr << uniform.first << " = gua_object_data[gua_draw_index]." << uniform.first + ";" << std::endl;
  }

  gua::string_utils::replace(source, "@material_input", sstr.str());

  #else
  
  sstr << "uniform mat4 gua_model_matrix;" << std::endl;
  sstr << "uniform mat4 gua_normal_matrix;" << std::endl;

  for (auto const& uniform: get_default_material().get_uniforms()) {
    sstr << "uniform " << uniform.second.get().get_glsl_type() << " "
         << uniform.first << ";" << std::endl;
  }
  sstr << std::endl;

  // insert uniforms
  gua::string_utils::replace(source, "@material_uniforms", sstr.str());
  gua::string_utils::replace(source, "@material_input", "");

  #endif

  sstr.str("");

  // material methods ----------------------------------------------------------
  for (auto& method: methods) {
    sstr << method.get_source() << std::endl;
  }
  gua::string_utils::replace(source, "@material_method_declarations", sstr.str());
  sstr.str("");

  // material method calls -----------------------------------------------------
  for (auto& method: methods) {
    sstr << method.get_name() << "();" << std::endl;
  }
  gua::string_utils::replace(source, "@material_method_calls", sstr.str());

  // indent and return code ----------------------------------------------------
  return string_utils::format_code(source);
}

}
