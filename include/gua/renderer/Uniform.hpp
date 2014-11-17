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

namespace gua {

class UniformValue {

  union Data {
    int int_;
    bool bool_;
    float float_;

    math::mat3 mat3_;
    math::mat4 mat4_;

    math::vec2 vec2_;
    math::vec3 vec3_;
    math::vec4 vec4_;

    math::vec2i vec2i_;
    math::vec3i vec3i_;
    math::vec4i vec4i_;

    math::vec2ui vec2ui_;
    math::vec3ui vec3ui_;
    math::vec4ui vec4ui_;

    std::string texture_;

    Data() : mat4_() {}
    Data(Data const& copy) : mat4_(copy.mat4_) {}
    ~Data() {  }

    Data& operator=(Data const& other) {
      mat4_ = other.mat4_;
      return *this;
    }
  };

  public:
    UniformValue() = default;

    // -------------------------------------------------------------------------
    template<typename T>
    UniformValue(T const& val) :
      apply_impl_(apply<T>),
      get_glsl_type_impl_(get_glsl_type_impl<T>),
      get_byte_size_impl_(get_byte_size_impl<T>),
      write_bytes_impl_(write_bytes_impl<T>) { set(val); }

    UniformValue(UniformValue const& to_copy) = default;
    // UniformValue(UniformValue const& to_copy) :
    //   apply_impl_(to_copy.apply_impl_),
    //   get_glsl_type_impl_(to_copy.get_glsl_type_impl_),
    //   get_byte_size_impl_(to_copy.get_byte_size_impl_),
    //   write_bytes_impl_(to_copy.write_bytes_impl_),
    //   data(to_copy.data) {}

    // -------------------------------------------------------------------------
    static UniformValue create_from_string_and_type(
      std::string const& value, UniformType const& ty
    );

    static UniformValue create_from_strings(
      std::string const& value, std::string const& ty
    );

    // -------------------------------------------------------------------------
    void apply(RenderContext const& ctx, std::string const& name,
               scm::gl::program_ptr const& prog, unsigned location = 0) const {
      apply_impl_(this, ctx, name, prog, location);
    }

    std::string get_glsl_type() const {
      return get_glsl_type_impl_();
    }

    unsigned get_byte_size() const {
      return get_byte_size_impl_();
    }

    void write_bytes(RenderContext const& ctx, char* target) const {
      write_bytes_impl_(this, ctx, target);
    }

    void operator= (UniformValue const& to_copy) {
      apply_impl_         = to_copy.apply_impl_;
      get_glsl_type_impl_ = to_copy.get_glsl_type_impl_;
      get_byte_size_impl_ = to_copy.get_byte_size_impl_;
      write_bytes_impl_   = to_copy.write_bytes_impl_;
      data                = to_copy.data;
    }

    Data data;

  private:

    void set(int val) { data.int_ = val; }
    void set(bool val) { data.bool_ = val; }
    void set(float val) { data.float_ = val; }
    void set(math::mat3 const& val) { data.mat3_ = val; }
    void set(math::mat4 const& val) { data.mat4_ = val; }
    void set(math::vec2 const& val) { data.vec2_ = val; }
    void set(math::vec3 const& val) { data.vec3_ = val; }
    void set(math::vec4 const& val) { data.vec4_ = val; }
    void set(math::vec2i const& val) { data.vec2i_ = val; }
    void set(math::vec3i const& val) { data.vec3i_ = val; }
    void set(math::vec4i const& val) { data.vec4i_ = val; }
    void set(math::vec2ui const& val) { data.vec2ui_ = val; }
    void set(math::vec3ui const& val) { data.vec3ui_ = val; }
    void set(math::vec4ui const& val) { data.vec4ui_ = val; }
    void set(std::string const& val) { new (&data.texture_) std::string; data.texture_ = val; }

    template<typename T>
    static void apply(UniformValue const* self, RenderContext const& ctx,
                      std::string const& name, scm::gl::program_ptr const& prog,
                      unsigned location);

    template<typename T>
    static std::string get_glsl_type_impl();

    template<typename T>
    static unsigned get_byte_size_impl();

    template<typename T>
    static void write_bytes_impl(UniformValue const* self, RenderContext const& ctx, char* target);

    std::function<void(UniformValue const*, RenderContext const&, std::string const&, scm::gl::program_ptr const&, unsigned)> apply_impl_;
    std::function<std::string()> get_glsl_type_impl_;
    std::function<unsigned()> get_byte_size_impl_;
    std::function<void(UniformValue const*, RenderContext const&, char*)> write_bytes_impl_;
};

}

#endif  // GUA_UNIFORM_HPP
