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

template<> void UniformValue::apply<std::string>(UniformValue const* self, RenderContext const& ctx, std::string const& name, scm::gl::program_ptr const& prog, unsigned location) {

  auto texture(TextureDatabase::instance()->lookup(boost::get<std::string>(self->data)));
  if (texture) {
    prog->uniform(name, location, texture->get_handle(ctx));
  } else if (ctx.mode != CameraMode::CENTER) {
    if ((ctx.mode != CameraMode::LEFT)) {
      auto left_texture(TextureDatabase::instance()->lookup(boost::get<std::string>(self->data) + "_left"));
      if (left_texture) {
        prog->uniform(name, location, left_texture->get_handle(ctx));
      }
    } else {
      auto right_texture(TextureDatabase::instance()->lookup(boost::get<std::string>(self->data) + "_right"));
      if (right_texture) {
        prog->uniform(name, location, right_texture->get_handle(ctx));
      }
    }   
  }
}

template<> unsigned UniformValue::get_byte_size_impl<bool>        () { return sizeof(int); }
template<> unsigned UniformValue::get_byte_size_impl<std::string> () { return sizeof(math::vec2ui); }


template<> void UniformValue::write_bytes_impl<bool>(UniformValue const* self, RenderContext const& ctx, char* target) { memcpy(target, &boost::get<bool>(self->data), sizeof(int)); }

template<> void UniformValue::write_bytes_impl<std::string> (UniformValue const* self, RenderContext const& ctx, char* target)
{
  auto texture(TextureDatabase::instance()->lookup(boost::get<std::string>(self->data)));
  if (texture) {
    auto& handle(texture->get_handle(ctx));
    memcpy(target, &handle, sizeof(math::vec2ui));
  } else if (ctx.mode != CameraMode::CENTER) {
    if ((ctx.mode != CameraMode::LEFT)) {
      auto left_texture(TextureDatabase::instance()->lookup(boost::get<std::string>(self->data) + "_left"));
      if (left_texture) {
        auto& handle(left_texture->get_handle(ctx));
        memcpy(target, &handle, sizeof(math::vec2ui));
      }
    } else {
      auto right_texture(TextureDatabase::instance()->lookup(boost::get<std::string>(self->data) + "_right"));
      if (right_texture) {
        auto& handle(right_texture->get_handle(ctx));
        memcpy(target, &handle, sizeof(math::vec2ui));
      }
    }   
  }
}

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
  throw std::runtime_error("UniformValue::create_from_string_and_type(): Invalid type");
}

UniformValue UniformValue::create_from_strings(
    std::string const& value,
    std::string const& ty) {

  return create_from_string_and_type(value,
                                     gua::enums::parse_uniform_type(ty).get()
                                    );

}

}
