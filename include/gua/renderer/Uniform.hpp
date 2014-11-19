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

class GUA_DLL UniformValue {

  typedef boost::variant < 
    int, 
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
    std::string
  > Data;

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

    template<typename T>
    void set(T const& val) 
    { 
      data = val; 
    }

    template<typename T>
    static void apply(UniformValue const* self, RenderContext const& ctx,
      std::string const& name, scm::gl::program_ptr const& prog,
      unsigned location)
    {
      prog->uniform(name, location, boost::get<T>(self->data));
    }

    template<typename T>
    static unsigned get_byte_size_impl()
    {
      return sizeof(T);
    }

    template<typename T>
    static void write_bytes_impl(UniformValue const* self, RenderContext const& ctx, char* target)
    {
      memcpy(target, &boost::get<T>(self->data), sizeof(T));
    }

    template<typename T>
    static std::string get_glsl_type_impl();


    std::function<void(UniformValue const*, RenderContext const&, std::string const&, scm::gl::program_ptr const&, unsigned)> apply_impl_;
    std::function<std::string()> get_glsl_type_impl_;
    std::function<unsigned()> get_byte_size_impl_;
    std::function<void(UniformValue const*, RenderContext const&, char*)> write_bytes_impl_;
};

// specializations
template<> void UniformValue::apply<std::string>(UniformValue const* self, RenderContext const& ctx, std::string const& name, scm::gl::program_ptr const& prog, unsigned location);

template<> void UniformValue::write_bytes_impl<std::string>(UniformValue const* self, RenderContext const& ctx, char* target);
template<> void UniformValue::write_bytes_impl<bool>(UniformValue const* self, RenderContext const& ctx, char* target);

template<> unsigned UniformValue::get_byte_size_impl<bool>();
template<> unsigned UniformValue::get_byte_size_impl<std::string>();
}                                                                                                                                                                                                                                                             

#endif  // GUA_UNIFORM_HPP
