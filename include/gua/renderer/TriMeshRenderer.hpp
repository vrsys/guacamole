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

#ifndef GUA_TRIMESH_RENDERER_HPP
#define GUA_TRIMESH_RENDERER_HPP

#include <map>
#include <unordered_map>

#include <gua/platform.hpp>

#include <scm/gl_core/shader_objects.h>

namespace gua {

  class MaterialShader;
  class ShaderProgram;
  class Pipeline;

class TriMeshRenderer {

 public:

   TriMeshRenderer();
   ~TriMeshRenderer();
  
   void render(Pipeline& pipe);

 private:

   std::map<scm::gl::shader_stage, std::string>         program_description_;
   std::unordered_map<MaterialShader*, ShaderProgram*>  programs_;
  
};

}

#endif  // GUA_TRIMESH_RENDERER_HPP
