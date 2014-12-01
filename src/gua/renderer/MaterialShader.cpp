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

#include <gua/databases/Resources.hpp>
#include <gua/utils/string_utils.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
MaterialShader::MaterialShader(std::string const& name, MaterialShaderDescription const& desc)
  : desc_(desc),
    default_material_(std::make_shared<Material>(name)),
    max_object_count_(0)
{
  auto v_methods = desc_.get_vertex_methods();
  auto f_methods = desc_.get_fragment_methods();

  for (auto const& method : v_methods) {
    for (auto const& uniform : method.get_uniforms()) {
      default_material_->set_uniform(uniform.first, uniform.second);
    }
  }

  for (auto const& method : f_methods) {
    for (auto const& uniform : method.get_uniforms()) {
      default_material_->set_uniform(uniform.first, uniform.second);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
MaterialShaderDescription const& MaterialShader::get_description() const {
  return desc_;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& MaterialShader::get_name() const {
  return default_material_->get_shader_name();
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> MaterialShader::get_new_material() const {
  return std::make_shared<Material>(default_material_->get_shader_name());
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> const& MaterialShader::get_default_material() const {
  return default_material_;
}

////////////////////////////////////////////////////////////////////////////////
ShaderProgram* MaterialShader::get_shader(std::map<scm::gl::shader_stage, std::string> const& program_description,
                                          std::list<std::string> const& interleaved_stream_capture,
                                          bool in_rasterization_discard)  
{
  // std::type_index type_id(typeid(for_type));
  using namespace scm::gl;

  std::vector<ShaderProgramStage> final_program_description;
  auto new_shader = new ShaderProgram();

  auto v_methods = desc_.get_vertex_methods();
  auto f_methods = desc_.get_fragment_methods();

  for (auto const& stage : program_description)
  {
    // insert material code in vertex and fragment shader
    if (stage.first == STAGE_VERTEX_SHADER) {
      auto v_shader(compile_description(v_methods, program_description.at(STAGE_VERTEX_SHADER)));
      final_program_description.push_back(ShaderProgramStage(STAGE_VERTEX_SHADER, v_shader));
    }
    else {
      if (stage.first == STAGE_GEOMETRY_SHADER) {
        auto g_shader(compile_description(v_methods, program_description.at(STAGE_GEOMETRY_SHADER)));
        final_program_description.push_back(ShaderProgramStage(STAGE_GEOMETRY_SHADER, g_shader));
      }
      else {
        if (stage.first == STAGE_FRAGMENT_SHADER) {
          auto f_shader(compile_description(f_methods, program_description.at(STAGE_FRAGMENT_SHADER)));
          final_program_description.push_back(ShaderProgramStage(STAGE_FRAGMENT_SHADER, f_shader));
        }
        else {
          // keep code for other shading stages
          final_program_description.push_back(ShaderProgramStage(stage.first, stage.second));
        }
      }
    }
  }
    
  new_shader->set_shaders(final_program_description, interleaved_stream_capture, in_rasterization_discard);
  return new_shader;
}


////////////////////////////////////////////////////////////////////////////////
void MaterialShader::apply_uniforms(RenderContext const& ctx,
                                    ShaderProgram* shader,
                                    int view,
                                    std::shared_ptr<Material> const& overwrite) const {

  // TODO: make this iterations thread-safe
  for (auto const& uniform : default_material_->get_uniforms()) {
    uniform.second.apply(ctx, uniform.first, view, shader->get_program(ctx));
  }

  for (auto const& uniform : overwrite->get_uniforms()) {
    uniform.second.apply(ctx, uniform.first, view, shader->get_program(ctx));
  }
}

////////////////////////////////////////////////////////////////////////////////

void MaterialShader::print_shaders() const {
  // for (auto shader: shaders_) {
  //   shader.second->print_shaders();
  // }
}


////////////////////////////////////////////////////////////////////////////////
std::string MaterialShader::compile_description(std::list<MaterialShaderMethod> const& methods,
                                                std::string const& shader_source) const {
  std::string source(shader_source);
  std::stringstream sstr;

  for (auto const& uniform: get_default_material()->get_uniforms()) {
    sstr << "uniform " << uniform.second.get().get_glsl_type() << " "
         << uniform.first << ";" << std::endl;
  }
  sstr << std::endl;

  // insert uniforms
  gua::string_utils::replace(source, "@material_uniforms", sstr.str());
  gua::string_utils::replace(source, "@material_input", "");

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
