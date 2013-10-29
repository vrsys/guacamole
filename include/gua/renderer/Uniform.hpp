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
#include <gua/renderer/Texture2D.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/utils/Color3f.hpp>
#include <gua/utils/logger.hpp>

// external headers
#include <string>
#include <scm/gl_core/shader_objects/program.h>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

namespace gua {

template <typename T> class UniformValue;

/**
 * Stores information on an Uniform.
 *
 */
class UniformValueBase {
 public:
  /**
   * Constructor.
   *
   */
  UniformValueBase() {}

  /**
   * Destructor.
   *
   */
  virtual ~UniformValueBase() {}

  virtual void apply(RenderContext const& context,
                     scm::gl::program_ptr program,
                     std::string const& name,
                     unsigned position = 0) const = 0;

  template <typename T> void set_value(T const& value) {

    auto casted(dynamic_cast<UniformValue<T>*>(this));

    if (casted) {
      casted->value(value);
    } else {
      WARNING("Unable to set uniform value: Types do not match!");
    }
  }

  template <typename T> T const& get_value() const {

    auto casted(dynamic_cast<UniformValue<T> const*>(this));

    if (casted) {
      return casted->value();
    } else {
      WARNING("Unable to get value of uniform: Types do not match!");
    }
  }
};



template <typename T> class UniformValue : public UniformValueBase {
 public:
  UniformValue(T const& value) : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    program->uniform(name, position, value_);
  }

  T const& value() const { return value_; }

  void value(T const& value) { value_ = value; }

 private:
  T value_;
};



template <>
class UniformValue<std::shared_ptr<Texture2D> > : public UniformValueBase {
 public:
  UniformValue(std::shared_ptr<Texture2D> const& value)
      : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    program->uniform(name, position, value_->get_handle(context));
  }

  std::shared_ptr<Texture2D> const& value() const { return value_; }

  void value(std::shared_ptr<Texture2D> const& value) { value_ = value; }

 private:
  std::shared_ptr<Texture2D> value_;
};

template <>
class UniformValue<Texture2D*> : public UniformValueBase {
 public:
  UniformValue(Texture2D* value)
      : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    program->uniform(name, position, value_->get_handle(context));
  }

  Texture2D* value() const { return value_; }

  void value(Texture2D* value) { value_ = value; }

 private:
  Texture2D* value_;
};



template <> class UniformValue<std::string> : public UniformValueBase {
 public:
  UniformValue(std::string const& value) : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    auto texture(TextureDatabase::instance()->lookup(value_));
    program->uniform(name, position, texture->get_handle(context));
  }

  std::string const& value() const { return value_; }

  void value(std::string const& value) { value_ = value; }

 private:
  std::string value_;
};



template <> class UniformValue<utils::Color3f> : public UniformValueBase {
 public:
  UniformValue(utils::Color3f const& value)
      : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    program->uniform(name, position, value_.vec3());
  }

  utils::Color3f const& value() const { return value_; }

  void value(utils::Color3f const& value) { value_ = value; }

 private:
  utils::Color3f value_;
};

}

#endif  // GUA_UNIFORM_HPP
