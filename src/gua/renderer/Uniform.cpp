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

template<> std::string UniformValue::get_glsl_type_impl<int>()          { return "int"; }
template<> std::string UniformValue::get_glsl_type_impl<bool>()         { return "int"; }
template<> std::string UniformValue::get_glsl_type_impl<float>()        { return "float"; }
template<> std::string UniformValue::get_glsl_type_impl<math::mat3>()   { return "mat3"; }
template<> std::string UniformValue::get_glsl_type_impl<math::mat4>()   { return "mat4"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec2>()   { return "vec2"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec3>()   { return "vec3"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec4>()   { return "vec4"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec2i>()  { return "ivec2"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec3i>()  { return "ivec3"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec4i>()  { return "ivec4"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec2ui>() { return "uvec2"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec3ui>() { return "uvec3"; }
template<> std::string UniformValue::get_glsl_type_impl<math::vec4ui>() { return "uvec4"; }
template<> std::string UniformValue::get_glsl_type_impl<std::string>()  { return "uvec2"; }

template<> void UniformValue::apply<int>          (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.int_); }
template<> void UniformValue::apply<bool>         (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.bool_); }
template<> void UniformValue::apply<float>        (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.float_); }
template<> void UniformValue::apply<math::mat3>   (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.mat3_); }
template<> void UniformValue::apply<math::mat4>   (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.mat4_); }
template<> void UniformValue::apply<math::vec2>   (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.vec2_); }
template<> void UniformValue::apply<math::vec3>   (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.vec3_); }
template<> void UniformValue::apply<math::vec4>   (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.vec4_); }
template<> void UniformValue::apply<math::vec2i>  (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.vec2i_); }
template<> void UniformValue::apply<math::vec3i>  (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.vec3i_); }
template<> void UniformValue::apply<math::vec4i>  (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.vec4i_); }
template<> void UniformValue::apply<math::vec2ui> (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.vec2ui_); }
template<> void UniformValue::apply<math::vec3ui> (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.vec3ui_); }
template<> void UniformValue::apply<math::vec4ui> (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { prog->uniform(self->get_name(), location, self->val_.vec4ui_); }
template<> void UniformValue::apply<std::string>  (UniformValue const* self, RenderContext const& ctx, scm::gl::program_ptr const& prog, unsigned location) { 
  auto texture(TextureDatabase::instance()->lookup(self->val_.texture_));
  if (texture) {
    prog->uniform(self->get_name(), location, texture->get_handle(ctx));
  }
}

template<> unsigned UniformValue::write_bytes_impl<int>         (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.int_,     sizeof(int));           return sizeof(int); }
template<> unsigned UniformValue::write_bytes_impl<bool>        (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.bool_,    sizeof(bool));          return sizeof(bool); }
template<> unsigned UniformValue::write_bytes_impl<float>       (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.float_,   sizeof(float));         return sizeof(float); }
template<> unsigned UniformValue::write_bytes_impl<math::mat3>  (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.mat3_,    sizeof(math::mat3));    return sizeof(math::mat3); }
template<> unsigned UniformValue::write_bytes_impl<math::mat4>  (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.mat4_,    sizeof(math::mat4));    return sizeof(math::mat4); }
template<> unsigned UniformValue::write_bytes_impl<math::vec2>  (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.vec2_,    sizeof(math::vec2));    return sizeof(math::vec2); }
template<> unsigned UniformValue::write_bytes_impl<math::vec3>  (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.vec3_,    sizeof(math::vec3));    return sizeof(math::vec3); }
template<> unsigned UniformValue::write_bytes_impl<math::vec4>  (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.vec4_,    sizeof(math::vec4));    return sizeof(math::vec4); }
template<> unsigned UniformValue::write_bytes_impl<math::vec2i> (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.vec2i_,   sizeof(math::vec2i));   return sizeof(math::vec2i); }
template<> unsigned UniformValue::write_bytes_impl<math::vec3i> (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.vec3i_,   sizeof(math::vec3i));   return sizeof(math::vec3i); }
template<> unsigned UniformValue::write_bytes_impl<math::vec4i> (UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.vec4i_,   sizeof(math::vec4i));   return sizeof(math::vec4i); }
template<> unsigned UniformValue::write_bytes_impl<math::vec2ui>(UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.vec2ui_,  sizeof(math::vec2ui));  return sizeof(math::vec2ui); }
template<> unsigned UniformValue::write_bytes_impl<math::vec3ui>(UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.vec3ui_,  sizeof(math::vec3ui));  return sizeof(math::vec3ui); }
template<> unsigned UniformValue::write_bytes_impl<math::vec4ui>(UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &self->val_.vec4ui_,  sizeof(math::vec4ui));  return sizeof(math::vec4ui); }
template<> unsigned UniformValue::write_bytes_impl<std::string> (UniformValue const* self, RenderContext const& ctx, char* target) { 
  auto texture(TextureDatabase::instance()->lookup(self->val_.texture_));
  if (texture) {
    auto& handle(texture->get_handle(ctx));
    memcpy(target, &handle, sizeof(math::vec2ui));
  }
  return sizeof(math::vec2ui);
}


UniformValue UniformValue::create_from_string_and_type(
    std::string const& name,
    std::string const& value,
    UniformType const& ty) {
  switch (ty) {
    case UniformType::INT:
      return UniformValue(name, string_utils::from_string<int>(value));
    case UniformType::FLOAT:
      return UniformValue(name, string_utils::from_string<float>(value));
    case UniformType::BOOL:
      return UniformValue(name, string_utils::from_string<bool>(value));
    case UniformType::VEC2:
      return UniformValue(name, string_utils::from_string<math::vec2>(value));
    case UniformType::VEC3:
      return UniformValue(name, string_utils::from_string<math::vec3>(value));
    case UniformType::VEC4:
      return UniformValue(name, string_utils::from_string<math::vec4>(value));
    case UniformType::MAT3:
      return UniformValue(name, string_utils::from_string<math::mat3>(value));
    case UniformType::MAT4:
      return UniformValue(name, string_utils::from_string<math::mat4>(value));
    case UniformType::SAMPLER2D:
      return UniformValue(name, value);
  }
}

UniformValue UniformValue::create_from_strings(
    std::string const& name,
    std::string const& value,
    std::string const& ty) {

  return create_from_string_and_type(name, value,
                                     gua::enums::parse_uniform_type(ty).get()
                                    );

}

}
