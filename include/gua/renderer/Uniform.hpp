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

  virtual std::vector<char> get_as_data(RenderContext const& context) = 0;

  template <typename T> void set_value(T const& value) {

    auto casted(dynamic_cast<UniformValue<T>*>(this));

    if (casted) {
      casted->value(value);
    } else {
      Logger::LOG_WARNING << "Unable to set uniform value: Types do not match!" << std::endl;
    }
  }

  template <typename T> T const& get_value() const {

    auto casted(dynamic_cast<UniformValue<T> const*>(this));

    if (casted) {
      return casted->value();
    } else {
      Logger::LOG_WARNING << "Unable to get value of uniform: Types do not match!" << std::endl;
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

  std::vector<char> get_as_data(RenderContext const& context) {
    return std::vector<char>(reinterpret_cast<char*>(&value_), reinterpret_cast<char*>(&value_) + sizeof(T));
  }

  T const& value() const { return value_; }

  void value(T const& value) { value_ = value; }

 private:
  T value_;
};



template <>
class UniformValue<std::shared_ptr<Texture> > : public UniformValueBase {
 public:
  UniformValue(std::shared_ptr<Texture> const& value)
      : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    program->uniform(name, position, value_->get_handle(context));
  }

  std::vector<char> get_as_data(RenderContext const& context) {
    auto handle(value_->get_handle(context));
    return std::vector<char>(reinterpret_cast<char*>(&handle), reinterpret_cast<char*>(&handle) + sizeof(math::vec2ui));
  }

  std::shared_ptr<Texture> const& value() const { return value_; }

  void value(std::shared_ptr<Texture> const& value) { value_ = value; }

 private:
  std::shared_ptr<Texture> value_;
};

template <>
class UniformValue<Texture*> : public UniformValueBase {
 public:
  UniformValue(Texture* value)
      : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    program->uniform(name, position, value_->get_handle(context));
  }

  std::vector<char> get_as_data(RenderContext const& context) {
    auto handle(value_->get_handle(context));
    return std::vector<char>(reinterpret_cast<char*>(&handle), reinterpret_cast<char*>(&handle) + sizeof(math::vec2ui));
  }

  Texture* value() const { return value_; }

  void value(Texture* value) { value_ = value; }

 private:
  Texture* value_;
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

  std::vector<char> get_as_data(RenderContext const& context) {
    auto handle(value_->get_handle(context));
    return std::vector<char>(reinterpret_cast<char*>(&handle), reinterpret_cast<char*>(&handle) + sizeof(math::vec2ui));
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

  std::vector<char> get_as_data(RenderContext const& context) {
    auto handle(value_->get_handle(context));
    return std::vector<char>(reinterpret_cast<char*>(&handle), reinterpret_cast<char*>(&handle) + sizeof(math::vec2ui));
  }

  Texture2D* value() const { return value_; }

  void value(Texture2D* value) { value_ = value; }

 private:
  Texture2D* value_;
};

template <>
class UniformValue<std::shared_ptr<Texture3D> > : public UniformValueBase {
 public:
  UniformValue(std::shared_ptr<Texture3D> const& value)
      : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    program->uniform(name, position, value_->get_handle(context));
  }

  std::vector<char> get_as_data(RenderContext const& context) {
    auto handle(value_->get_handle(context));
    return std::vector<char>(reinterpret_cast<char*>(&handle), reinterpret_cast<char*>(&handle) + sizeof(math::vec2ui));
  }

  std::shared_ptr<Texture3D> const& value() const { return value_; }

  void value(std::shared_ptr<Texture3D> const& value) { value_ = value; }

 private:
  std::shared_ptr<Texture3D> value_;
};

template <>
class UniformValue<Texture3D*> : public UniformValueBase {
 public:
  UniformValue(Texture3D* value)
      : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    program->uniform(name, position, value_->get_handle(context));
  }

  std::vector<char> get_as_data(RenderContext const& context) {
    auto handle(value_->get_handle(context));
    return std::vector<char>(reinterpret_cast<char*>(&handle), reinterpret_cast<char*>(&handle) + sizeof(math::vec2ui));
  }

  Texture3D* value() const { return value_; }

  void value(Texture3D* value) { value_ = value; }

 private:
  Texture3D* value_;
};

template <> class UniformValue<std::string> : public UniformValueBase {
 public:
  UniformValue(std::string const& value) : UniformValueBase(), value_(value) {}

  void apply(RenderContext const& context,
             scm::gl::program_ptr program,
             std::string const& name,
             unsigned position = 0) const {

    auto texture(TextureDatabase::instance()->lookup(value_));
    if (texture) {
      program->uniform(name, position, texture->get_handle(context));
    }
  }

  std::vector<char> get_as_data(RenderContext const& context) {
    auto texture(TextureDatabase::instance()->lookup(value_));
    if (texture) {
      auto handle(texture->get_handle(context));
      return std::vector<char>(reinterpret_cast<char*>(&handle), reinterpret_cast<char*>(&handle) + sizeof(math::vec2ui));
    }

    return std::vector<char>();
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

  std::vector<char> get_as_data(RenderContext const& context) {
    math::vec3 tmp(value_.vec3());
    return std::vector<char>(reinterpret_cast<char*>(&tmp), reinterpret_cast<char*>(&tmp) + sizeof(math::vec3));
  }

  utils::Color3f const& value() const { return value_; }

  void value(utils::Color3f const& value) { value_ = value; }

 private:
  utils::Color3f value_;
};

}

#endif  // GUA_UNIFORM_HPP
