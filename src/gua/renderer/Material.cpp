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
#include <gua/renderer/Material.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>
#include <gua/memory.hpp>

// external headers
#include <sstream>

namespace gua {

std::unique_ptr<UniformValueBase> create_from_string_and_type(
    std::string const& value,
    UniformType const& ty) {
  switch (ty) {
    case UniformType::INT:
      return gua::make_unique<UniformValue<int> >(
          string_utils::from_string<int>(value));
      break;

    case UniformType::FLOAT:
      return gua::make_unique<UniformValue<float> >(
          string_utils::from_string<float>(value));
      break;

    case UniformType::BOOL:
      return gua::make_unique<UniformValue<bool> >(
          string_utils::from_string<bool>(value));
      break;

    case UniformType::VEC2:
      return gua::make_unique<UniformValue<math::vec2> >(
          string_utils::from_string<math::vec2>(value));
      break;

    case UniformType::VEC3:
      return gua::make_unique<UniformValue<math::vec3> >(
          string_utils::from_string<math::vec3>(value));
      break;

    case UniformType::VEC4:
      return gua::make_unique<UniformValue<math::vec4> >(
          string_utils::from_string<math::vec4>(value));
      break;

    case UniformType::MAT3:
      return gua::make_unique<UniformValue<math::mat3> >(
          string_utils::from_string<math::mat3>(value));
      break;

    case UniformType::MAT4:
      return gua::make_unique<UniformValue<math::mat4> >(
          string_utils::from_string<math::mat4>(value));
      break;

    case UniformType::SAMPLER2D:
      return gua::make_unique<UniformValue<std::string> >(value);
      break;

    default:
      return nullptr;
  }
}

unsigned Material::global_id_count_ = 0;

Material::Material()
    : uniform_values_(), name_("Unnamed Material"), description_() {}

Material::Material(std::string const& name)
    : uniform_values_(), name_(name), description_() {}

Material::Material(std::string const& name,
                   MaterialDescription const& description)
    : uniform_values_(),
      name_(name),
      id_(++global_id_count_),
      description_(description) {

  load_description();
}

void Material::reload() {
  description_.reload();
  load_description();
}

void Material::load_description() {
  uniform_values_.clear();

  std::string shading_model(description_.get_shading_model());
  std::shared_ptr<ShadingModel> mod;

  if (!ShadingModelDatabase::instance()->is_supported(shading_model)) {
    mod = std::make_shared<ShadingModel>(shading_model, shading_model);
    ShadingModelDatabase::instance()->add(shading_model, mod);
  } else {
    mod = ShadingModelDatabase::instance()->lookup(shading_model);
  }

  for (auto& stage : mod->get_stages()) {
    for (auto const& uniform : stage.get_uniforms()) {
      std::string value(description_.get_uniforms()[uniform.first]);
      uniform_values_[uniform.first] =
          create_from_string_and_type(value, uniform.second);
    }
  }
}

}
