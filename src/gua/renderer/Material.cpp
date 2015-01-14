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

#include <gua/renderer/Material.hpp>

#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/renderer/ShaderProgram.hpp>
 
namespace gua {

////////////////////////////////////////////////////////////////////////////////

Material::Material(std::string const& shader_name):
  shader_name_(shader_name),
  shader_cache_(nullptr)
  {
    set_shader_name(shader_name_);
  }

////////////////////////////////////////////////////////////////////////////////

Material::Material(Material const& copy):
  shader_name_(copy.shader_name_),
  shader_cache_(copy.shader_cache_),
  uniforms_(copy.uniforms_)
  {}

////////////////////////////////////////////////////////////////////////////////

std::string const& Material::get_shader_name() const {
  return shader_name_;
}

////////////////////////////////////////////////////////////////////////////////

void Material::set_shader_name(std::string const& name) {
  boost::unique_lock<boost::shared_mutex> lock(mutex_);
  shader_name_ = name;
  shader_cache_ = nullptr;

  auto shader(MaterialShaderDatabase::instance()->lookup(shader_name_).get());
  auto new_uniforms(shader->get_default_uniforms());

  for (auto const& old_uniform : uniforms_) {
    auto it(new_uniforms.find(old_uniform.first));
    if (it != new_uniforms.end()) {
      it->second = old_uniform.second;
    }
  }
  

  uniforms_ = new_uniforms;
}

////////////////////////////////////////////////////////////////////////////////

MaterialShader* Material::get_shader() const {
  // boost::unique_lock<boost::shared_mutex> lock(mutex_);
  if (!shader_cache_) {
    shader_cache_ = MaterialShaderDatabase::instance()->lookup(shader_name_).get();
  }

  return shader_cache_;
}

////////////////////////////////////////////////////////////////////////////////

std::map<std::string, ViewDependentUniform> const& Material::get_uniforms() const {
  return uniforms_;
}

////////////////////////////////////////////////////////////////////////////////

void Material::apply_uniforms(RenderContext const& ctx, ShaderProgram* shader, int view) const {
    boost::unique_lock<boost::shared_mutex> lock(mutex_);
    for (auto const& uniform : uniforms_) {
        uniform.second.apply(ctx, uniform.first, view, shader->get_program(ctx));
    }
}

////////////////////////////////////////////////////////////////////////////////

}
