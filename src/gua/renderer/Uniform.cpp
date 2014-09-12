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
#include <gua/renderer/Uniform.hpp>

#include <gua/math/math.hpp>
#include <gua/memory.hpp>

namespace gua {

template<> std::string UniformValue<int>         ::get_glsl_type() const { return "int"; }
template<> std::string UniformValue<bool>        ::get_glsl_type() const { return "int"; }
template<> std::string UniformValue<float>       ::get_glsl_type() const { return "float"; }

template<> std::string UniformValue<math::mat3>  ::get_glsl_type() const { return "mat3"; }
template<> std::string UniformValue<math::mat4>  ::get_glsl_type() const { return "mat4"; }

template<> std::string UniformValue<math::vec2>  ::get_glsl_type() const { return "vec2"; }
template<> std::string UniformValue<math::vec3>  ::get_glsl_type() const { return "vec3"; }
template<> std::string UniformValue<math::vec4>  ::get_glsl_type() const { return "vec4"; }

template<> std::string UniformValue<math::vec2i> ::get_glsl_type() const { return "vec2i"; }
template<> std::string UniformValue<math::vec3i> ::get_glsl_type() const { return "vec3i"; }
template<> std::string UniformValue<math::vec4i> ::get_glsl_type() const { return "vec4i"; }

template<> std::string UniformValue<math::vec2ui>::get_glsl_type() const { return "vec2ui"; }
template<> std::string UniformValue<math::vec3ui>::get_glsl_type() const { return "vec3ui"; }
template<> std::string UniformValue<math::vec4ui>::get_glsl_type() const { return "vec4ui"; }

std::unique_ptr<UniformValueBase> UniformValueBase::create_from_string_and_type(
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

std::unique_ptr<UniformValueBase> UniformValueBase::create_from_strings(
    std::string const& value,
    std::string const& ty) {

  return create_from_string_and_type(value,
                                     gua::enums::parse_uniform_type(ty).get()
                                    );

}

}
