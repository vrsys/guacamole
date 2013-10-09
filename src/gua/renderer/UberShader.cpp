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
#include <gua/utils/logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

UberShader::UberShader() : uniform_mapping_(), output_mapping_() {}

////////////////////////////////////////////////////////////////////////////////

void UberShader::set_material_uniforms(std::set<std::string> const& materials,
                                       ShadingModel::StageID stage,
                                       RenderContext const& context) {

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
        apply_uniform(
            context, uniform->second.get(), mapped.first, mapped.second);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

LayerMapping const* UberShader::get_gbuffer_mapping() const {
  return &output_mapping_;
}

////////////////////////////////////////////////////////////////////////////////

UniformMapping const* UberShader::get_uniform_mapping() const {
  return &uniform_mapping_;
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

}
