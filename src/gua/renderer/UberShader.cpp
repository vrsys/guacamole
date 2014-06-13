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
#include <gua/renderer/UberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/memory.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

UberShader::UberShader()
: uniform_mapping_(),
  output_mapping_(),
  programs_()
{}

////////////////////////////////////////////////////////////////////////////////

UberShader::~UberShader()
{}

////////////////////////////////////////////////////////////////////////////////

void UberShader::create(std::set<std::string> const& material_names)
{

  programs_.clear();

  vshader_factory_ = gua::make_unique<UberShaderFactory>(
    ShadingModel::GBUFFER_VERTEX_STAGE, material_names
    );

  fshader_factory_ = gua::make_unique<UberShaderFactory>(
    ShadingModel::GBUFFER_FRAGMENT_STAGE, material_names,
    vshader_factory_->get_uniform_mapping()
    );

  LayerMapping vshader_output_mapping = vshader_factory_->get_output_mapping();

  fshader_factory_->add_inputs_to_main_functions(
    { &vshader_output_mapping },
    ShadingModel::GBUFFER_VERTEX_STAGE
  );

  UberShader::set_uniform_mapping(fshader_factory_->get_uniform_mapping());
  UberShader::set_output_mapping(fshader_factory_->get_output_mapping());
}

////////////////////////////////////////////////////////////////////////////////

void UberShader::set_material_uniforms(std::set<std::string> const& materials,
                                       ShadingModel::StageID stage,
                                       RenderContext const& context) {
  for (auto const& program : programs_) {
    for (auto const& mat_name : materials) {
      auto const& material(MaterialDatabase::instance()->lookup(mat_name));
      auto const& shading_model(ShadingModelDatabase::instance()->lookup(
        material->get_description().get_shading_model()));

      for (auto const& uniform_name :
        shading_model->get_stages()[stage].get_uniforms()) {
        auto const& mapped(
          uniform_mapping_.get_mapping(mat_name, uniform_name.first));
        auto const& uniform(
          material->get_uniform_values().find(uniform_name.first));

        if (uniform != material->get_uniform_values().end()) {
          program->apply_uniform(context, uniform->second.get(), mapped.first, mapped.second);
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

LayerMapping const* UberShader::get_gbuffer_mapping() const {
  return &fshader_factory_->get_output_mapping();
}


////////////////////////////////////////////////////////////////////////////////

UniformMapping const* UberShader::get_uniform_mapping() const {
  return &fshader_factory_->get_uniform_mapping();
}


////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void UberShader::add_program(std::shared_ptr<ShaderProgram> const& pre_pass)
{
  programs_.push_back(pre_pass);
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ std::shared_ptr<ShaderProgram> const& UberShader::get_program(unsigned pass) const
{
  assert(programs_.size() > pass);
  return programs_[pass];
}

////////////////////////////////////////////////////////////////////////////////

std::vector<std::shared_ptr<ShaderProgram>> const& UberShader::programs() const
{
  return programs_;
}

////////////////////////////////////////////////////////////////////////////////

void UberShader::cleanup(RenderContext const& context) {
  for (auto program : programs_) {
    if (program) program->unuse(context);
  }
}


////////////////////////////////////////////////////////////////////////////////

void UberShader::set_uniform_mapping(UniformMapping const& mapping) {
  uniform_mapping_ = mapping;
}

////////////////////////////////////////////////////////////////////////////////

void UberShader::set_output_mapping(LayerMapping const& mapping) {
  output_mapping_ = mapping;
}

////////////////////////////////////////////////////////////////////////////////

std::string const UberShader::print_material_switch(
    UberShaderFactory const& factory) const {

  std::stringstream s;

  auto main_calls(factory.get_main_calls());
  unsigned current_case(0);
  unsigned cases_per_block(CASES_PER_UBERSHADER_SWITCH);

  auto call(main_calls.begin());

  while (current_case < main_calls.size()) {

    s << "switch(gua_get_material_id()) {" << std::endl;

    for (unsigned i(current_case);
         i < main_calls.size() && i < current_case + cases_per_block;
         ++i) {
      s << " case " << call->first << ": " << call->second << " break;"
        << std::endl;
      ++call;
    }

    s << "}" << std::endl;

    current_case += cases_per_block;
  }

  return s.str();
}

////////////////////////////////////////////////////////////////////////////////

std::string const UberShader::print_material_methods(
    UberShaderFactory const& factory) const {

  std::stringstream s;

  for (auto const& method : factory.get_custom_function_declares()) {
    s << method << std::endl;
  }

  for (auto const& method : factory.get_custom_functions()) {
    s << method << std::endl;
  }

  for (auto const& method : factory.get_main_functions()) {
    s << method.second << std::endl;
  }

  return s.str();
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ bool UberShader::upload_to(RenderContext const& context) const
{
  bool upload_succeeded = true;

  for (auto const& program : programs_) {
    upload_succeeded &= program->upload_to(context);
  }

  return upload_succeeded;
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void UberShader::save_shaders_to_file(std::string const& directory,
                                                  std::string const& name) const {

  for (int i(0); i < programs_.size(); ++i) {
    programs_[i]->save_to_file(directory, name + string_utils::to_string(i));
  }
}

////////////////////////////////////////////////////////////////////////////////

}
