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
    
  public:
    ~UniformValue() { std::cout << "gone " << this << std::endl;}
    UniformValue() :
      val_(),
      apply_impl_([](RenderContext const& ctx, std::string const& name, scm::gl::program_ptr const& prog, unsigned location){
        std::cout << "muh" << std::endl;
      }),
      get_glsl_type_impl_([](){
        std::cout << "muh" << std::endl;
        return "muh";
      }) {}

    // -------------------------------------------------------------------------
    template<typename T>
    UniformValue(T const& val) :
      val_(val),
      apply_impl_([this](RenderContext const& ctx, std::string const& name, scm::gl::program_ptr const& prog, unsigned location){
        std::cout << "apply_impl_ " << this << std::endl;
        prog->uniform(name, location, boost::any_cast<T>(val_));
      }),
      get_glsl_type_impl_([this](){
        std::cout << "get_glsl_type_impl_ " << this << std::endl;
        return get_glsl_type_impl<T>();
      }) {}

    // -------------------------------------------------------------------------
    UniformValue(std::string const& val) :
      val_(val),
      apply_impl_([this](RenderContext const& ctx, std::string const& name, scm::gl::program_ptr const& prog, unsigned location){
        std::cout << "apply_impl_ " << this << std::endl;
        std::cout << get_glsl_type() << std::endl;
        auto texture(TextureDatabase::instance()->lookup(boost::any_cast<std::string>(val_)));
        if (texture) {
          prog->uniform(name, location, texture->get_handle(ctx));
        }
      }),
      get_glsl_type_impl_([this](){
        std::cout << "get_glsl_type_impl_ " << this << std::endl;
        return "uvec2";
      }) {}

    // -------------------------------------------------------------------------
    static UniformValue create_from_string_and_type(std::string const& value,
                                                    UniformType const& ty);

    static UniformValue create_from_strings(std::string const& value,
                                            std::string const& ty);

    // -------------------------------------------------------------------------
    void apply(RenderContext const& ctx,
               std::string const& name,
               scm::gl::program_ptr const& prog,
               unsigned location = 0) const {
        std::cout << "apply " << this << std::endl;
      if (apply_impl_) {
        apply_impl_(ctx, name, prog, location);
      }
    }

    std::string get_glsl_type() const {
        std::cout << "get_glsl_type " << this << std::endl;
      if (get_glsl_type_impl_) {
       return get_glsl_type_impl_();
      }
      return "muh";
    }

  private:
    template<typename T>
    std::string get_glsl_type_impl() const;

    boost::any  val_;
    std::function<void(RenderContext const& ctx, std::string const&, scm::gl::program_ptr const&, unsigned location)> apply_impl_;
    std::function<std::string()> get_glsl_type_impl_;
};

}

#endif  // GUA_UNIFORM_HPP
