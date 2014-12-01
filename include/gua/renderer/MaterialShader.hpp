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

#ifndef GUA_MATERIAL_SHADER_HPP
#define GUA_MATERIAL_SHADER_HPP

#include <gua/renderer/MaterialShaderDescription.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/string_utils.hpp>

#include <typeindex>
#include <sstream>
#include <iostream>

#define MATERIAL_SHADER_PROGRAMS 0

namespace gua {

class GUA_DLL MaterialShader {
 public:

  MaterialShader(std::string const& name, MaterialShaderDescription const& desc);

  MaterialShaderDescription const& get_description() const;

  std::string const&  get_name()             const;
  std::shared_ptr<Material>            get_new_material()     const;
  std::shared_ptr<Material> const&     get_default_material() const;

  ShaderProgram* get_shader(std::map<scm::gl::shader_stage, std::string> const& program_description,
                            std::list<std::string> const& interleaved_stream_capture = std::list<std::string>(),
                            bool in_rasterization_discard = false);

  void apply_uniforms(RenderContext const& ctx,
                      ShaderProgram* shader,
                      int view,
                      std::shared_ptr<Material> const& overwrite) const;


  void print_shaders() const;

  unsigned max_object_count() const {
    return max_object_count_;
  }

 private:

   std::string compile_description(std::list<MaterialShaderMethod> const& passes,
                                   std::string const& shader_source) const;

  MaterialShaderDescription desc_;

  std::shared_ptr<Material> default_material_;
  mutable unsigned max_object_count_;
};

}

#endif  // GUA_MATERIAL_SHADER_HPP
