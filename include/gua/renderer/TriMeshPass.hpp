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

#ifndef GUA_TRIMESH_PASS_HPP
#define GUA_TRIMESH_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <gua/platform.hpp>

// external headers
#include <scm/gl_core/buffer_objects.h>

namespace gua {

class Pipeline;
class TriMeshPass;

class GUA_DLL TriMeshPassDescription : public PipelinePassDescription {
 public:
  virtual PipelinePassDescription* make_copy() const;
  friend class Pipeline;

 protected:
  virtual PipelinePass* make_pass() const;
};



class GUA_DLL TriMeshPass : public PipelinePass {
 public:

  virtual bool needs_color_buffer_as_input() const { return false; }
  virtual bool writes_only_color_buffer()    const { return false; }

  virtual void process(PipelinePassDescription* desc, Pipeline* pipe);

  friend class TriMeshPassDescription;

 protected:
  TriMeshPass();
  ~TriMeshPass() {}

 private:
  std::string vertex_shader_;
  std::string fragment_shader_;

  mutable scm::gl::buffer_ptr material_uniform_storage_buffer_;
};

}

#endif  // GUA_TRIMESH_PASS_HPP
