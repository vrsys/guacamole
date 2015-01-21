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

#ifndef GUA_UNIFORM_HPP
#define GUA_UNIFORM_HPP

// guacamole headers
#include <gua/renderer/Texture.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/Texture3D.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/utils/Color3f.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/renderer/enums.hpp>

// external headers
#include <string>
#include <scm/gl_core/shader_objects/program.h>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

#include <boost/variant.hpp>

namespace gua {

struct GUA_DLL GetGlslType : public boost::static_visitor<std::string> {
  std::string operator()(int) const { return "int"; }
  std::string operator()(bool) const { return "int"; }
  std::string operator()(float) const { return "float"; }
  std::string operator()(math::mat3) const { return "mat3"; }
  std::string operator()(math::mat4) const { return "mat4"; }
  std::string operator()(math::vec2) const { return "vec2"; }
  std::string operator()(math::vec3) const { return "vec3"; }
  std::string operator()(math::vec4) const { return "vec4"; }
  std::string operator()(math::vec2i) const { return "ivec2"; }
  std::string operator()(math::vec3i) const { return "ivec3"; }
  std::string operator()(math::vec4i) const { return "ivec4"; }
  std::string operator()(math::vec2ui) const { return "uvec2"; }
  std::string operator()(math::vec3ui) const { return "uvec3"; }
  std::string operator()(math::vec4ui) const { return "uvec4"; }
  std::string operator()(std::string) const { return "uvec2"; }
};

struct GUA_DLL GetByteSize : public boost::static_visitor<unsigned> {
  unsigned operator()(int) const { return sizeof(int); }
  unsigned operator()(bool) const { return sizeof(int); }
  unsigned operator()(float) const { return sizeof(float); }
  unsigned operator()(math::mat3) const { return sizeof(math::mat3); }
  unsigned operator()(math::mat4) const { return sizeof(math::mat4); }
  unsigned operator()(math::vec2) const { return sizeof(math::vec2); }
  unsigned operator()(math::vec3) const { return sizeof(math::vec3); }
  unsigned operator()(math::vec4) const { return sizeof(math::vec4); }
  unsigned operator()(math::vec2i) const { return sizeof(math::vec2i); }
  unsigned operator()(math::vec3i) const { return sizeof(math::vec3i); }
  unsigned operator()(math::vec4i) const { return sizeof(math::vec4i); }
  unsigned operator()(math::vec2ui) const { return sizeof(math::vec2ui); }
  unsigned operator()(math::vec3ui) const { return sizeof(math::vec3ui); }
  unsigned operator()(math::vec4ui) const { return sizeof(math::vec4ui); }
  unsigned operator()(std::string) const { return sizeof(math::vec2ui); }
};


class GUA_DLL UniformValue {

  typedef boost::variant<int,
                         bool,
                         float,
                         math::mat3,
                         math::mat4,
                         math::vec2,
                         math::vec3,
                         math::vec4,
                         math::vec2i,
                         math::vec3i,
                         math::vec4i,
                         math::vec2ui,
                         math::vec3ui,
                         math::vec4ui,
                         std::string> Data;

 public:
  UniformValue() = default;

  // -------------------------------------------------------------------------
  template <typename T>
  UniformValue(T const& val)
      : apply_impl_(apply<T>)
      , write_bytes_impl_(write_bytes_impl<T>)
      , serialize_to_stream_impl_(serialize_to_stream_impl<T>) {
    set(val);
  }

  UniformValue(UniformValue const& to_copy) = default;
  // UniformValue(UniformValue const& to_copy) :
  //   apply_impl_(to_copy.apply_impl_),
  //   write_bytes_impl_(to_copy.write_bytes_impl_),
  //   data(to_copy.data) {}

  // -------------------------------------------------------------------------
  static UniformValue create_from_string_and_type(std::string const& value,
                                                  UniformType const& ty);

  static UniformValue create_from_strings(std::string const& value,
                                          std::string const& ty);

  static UniformValue create_from_serialized_string(std::string const& value);

  // -------------------------------------------------------------------------
  void apply(RenderContext const& ctx,
             std::string const& name,
             scm::gl::program_ptr const& prog,
             unsigned location = 0) const {
    apply_impl_(this, ctx, name, prog, location);
  }

  std::string get_glsl_type() const {
    return boost::apply_visitor(GetGlslType(), data);
  }

  unsigned get_byte_size() const {
    return boost::apply_visitor(GetByteSize(), data);
  }

  std::ostream& serialize_to_stream(std::ostream& os) const {
    return serialize_to_stream_impl_(this, os);
  }

  void write_bytes(RenderContext const& ctx, char* target) const {
    write_bytes_impl_(this, ctx, target);
  }

  void operator=(UniformValue const& to_copy) {
    apply_impl_ = to_copy.apply_impl_;
    write_bytes_impl_ = to_copy.write_bytes_impl_;
    serialize_to_stream_impl_ = to_copy.serialize_to_stream_impl_;
    data = to_copy.data;
  }

  Data data;

 private:

  template <typename T> void set(T const& val) { data = val; }

  template <typename T>
  static void apply(UniformValue const* self,
                    RenderContext const& ctx,
                    std::string const& name,
                    scm::gl::program_ptr const& prog,
                    unsigned location) {
    prog->uniform(name, location, boost::get<T>(self->data));
  }

  template <typename T>
  static void write_bytes_impl(UniformValue const* self,
                               RenderContext const& ctx,
                               char* target) {
    memcpy(target, &boost::get<T>(self->data), sizeof(T));
  }

  template <typename T>
  static std::ostream& serialize_to_stream_impl(UniformValue const* self,
                                                std::ostream& os) {
    return os;
  }

  std::function<void(UniformValue const*,
                     RenderContext const&,
                     std::string const&,
                     scm::gl::program_ptr const&,
                     unsigned)> apply_impl_;

  std::function<void(UniformValue const*, RenderContext const&, char*)>
      write_bytes_impl_;

  std::function<std::ostream& (UniformValue const*, std::ostream& os)>
      serialize_to_stream_impl_;
};

// specializations
template <>
GUA_DLL void UniformValue::apply<std::string>(UniformValue const* self,
                                              RenderContext const& ctx,
                                              std::string const& name,
                                              scm::gl::program_ptr const& prog,
                                              unsigned location);

template <>
GUA_DLL void UniformValue::write_bytes_impl<std::string>(
    UniformValue const* self,
    RenderContext const& ctx,
    char* target);

template <>
GUA_DLL void UniformValue::write_bytes_impl<bool>(UniformValue const* self,
                                                  RenderContext const& ctx,
                                                  char* target);


template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<int>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<bool>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<float>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::mat3>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::mat4>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::vec2>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::vec3>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::vec4>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::vec2i>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::vec3i>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::vec4i>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::vec2ui>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::vec3ui>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<math::vec4ui>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

template <>
GUA_DLL std::ostream& UniformValue::serialize_to_stream_impl<std::string>(
                                                       UniformValue const* self,
                                                       std::ostream& os);

//operators
std::ostream& operator<<(std::ostream& os, UniformValue const& val);

}

#endif  // GUA_UNIFORM_HPP
