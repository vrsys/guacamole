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

#include <gua/node/GeometryNode.hpp>

namespace gua {

template<> std::string UniformValue::get_glsl_type_impl<int>() const { return "int"; }
template<> std::string UniformValue::get_glsl_type_impl<bool>() const { return "int"; }
template<> std::string UniformValue::get_glsl_type_impl<float>() const { return "float"; }

template<> std::string UniformValue::get_glsl_type_impl<math::mat3>() const { return "mat3"; }
template<> std::string UniformValue::get_glsl_type_impl<math::mat4>() const { return "mat4"; }

template<> std::string UniformValue::get_glsl_type_impl<math::vec2>() const { return "vec2"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec3>() const { return "vec3"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec4>() const { return "vec4"; }

template<> std::string UniformValue::get_glsl_type_impl<math::vec2i>() const { return "ivec2"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec3i>() const { return "ivec3"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec4i>() const { return "ivec4"; }

template<> std::string UniformValue::get_glsl_type_impl<math::vec2ui>() const { return "uvec2"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec3ui>() const { return "uvec3"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec4ui>() const { return "uvec4"; }

UniformValue UniformValue::create_from_string_and_type(
    std::string const& value,
    UniformType const& ty) {
  switch (ty) {
    case UniformType::INT:
      return UniformValue(string_utils::from_string<int>(value));
    case UniformType::FLOAT:
      return UniformValue(string_utils::from_string<float>(value));
    case UniformType::BOOL:
      return UniformValue(string_utils::from_string<bool>(value));
    case UniformType::VEC2:
      return UniformValue(string_utils::from_string<math::vec2>(value));
    case UniformType::VEC3:
      return UniformValue(string_utils::from_string<math::vec3>(value));
    case UniformType::VEC4:
      return UniformValue(string_utils::from_string<math::vec4>(value));
    case UniformType::MAT3:
      return UniformValue(string_utils::from_string<math::mat3>(value));
    case UniformType::MAT4:
      return UniformValue(string_utils::from_string<math::mat4>(value));
    case UniformType::SAMPLER2D:
      return UniformValue(value);
  }
}

UniformValue UniformValue::create_from_strings(
    std::string const& value,
    std::string const& ty) {

  return create_from_string_and_type(value,
                                     gua::enums::parse_uniform_type(ty).get()
                                    );

}

}
