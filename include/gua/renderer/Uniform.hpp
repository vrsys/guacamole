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
    UniformValue(std::string const& name, T const& val) :
      name_(name),
      apply_impl_(apply<T>), 
      get_glsl_type_impl_(get_glsl_type_impl<T>) { set(val); }

    // -------------------------------------------------------------------------
    static UniformValue create_from_string_and_type(std::string const& name,
                                                    std::string const& value,
                                                    UniformType const& ty);

    static UniformValue create_from_strings(std::string const& name,
                                            std::string const& value,
                                            std::string const& ty);

    // -------------------------------------------------------------------------
    void apply(RenderContext const& ctx, scm::gl::program_ptr const& prog,
               unsigned location = 0) const {
      apply_impl_(this, ctx, prog, location);
    }

    std::string const& get_name() const { return name_; }
    

    std::string get_glsl_type() const {
      return get_glsl_type_impl_();
    }

  private:

    void set(int val) { val_.int_ = val; }
    void set(bool val) { val_.bool_ = val; }
    void set(float val) { val_.float_ = val; }
    void set(math::mat3 const& val) { val_.mat3_ = val; }
    void set(math::mat4 const& val) { val_.mat4_ = val; }
    void set(math::vec2 const& val) { val_.vec2_ = val; }
    void set(math::vec3 const& val) { val_.vec3_ = val; }
    void set(math::vec4 const& val) { val_.vec4_ = val; }
    void set(math::vec2i const& val) { val_.vec2i_ = val; }
    void set(math::vec3i const& val) { val_.vec3i_ = val; }
    void set(math::vec4i const& val) { val_.vec4i_ = val; }
    void set(math::vec2ui const& val) { val_.vec2ui_ = val; }
    void set(math::vec3ui const& val) { val_.vec3ui_ = val; }
    void set(math::vec4ui const& val) { val_.vec4ui_ = val; }
    void set(std::string const& val) { new (&val_.texture_) std::string; val_.texture_ = val; }
    
    template<typename T>
    static void apply(UniformValue const* self, RenderContext const& ctx,
                      scm::gl::program_ptr const& prog, unsigned location);

    template<typename T>
    static std::string get_glsl_type_impl();

    std::string name_;

    Data val_;
    std::function<void(UniformValue const*, RenderContext const& ctx, scm::gl::program_ptr const&, unsigned location)> apply_impl_;
    std::function<std::string()> get_glsl_type_impl_;
};

}

#endif  // GUA_UNIFORM_HPP
