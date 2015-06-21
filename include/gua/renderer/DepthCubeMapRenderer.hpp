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

#ifndef GUA_DEPTH_CUBEMAP_RENDERER_HPP
#define GUA_DEPTH_CUBEMAP_RENDERER_HPP

#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <scm/gl_core/shader_objects.h>

namespace gua {

class Pipeline;
class PipelinePassDescription;

class DepthCubeMapRenderer {

 public:

  enum Mode {
    COMPLETE            = 0,
    ONE_SIDE_PER_FRAME  = 1
  };

  DepthCubeMapRenderer();
  virtual ~DepthCubeMapRenderer() {}

  void render(Pipeline& pipe, PipelinePassDescription const& desc);

  void set_global_substitution_map(SubstitutionMap const& smap) { global_substitution_map_ = smap; }

  void create_state_objects(RenderContext const& ctx);

 private:

  Mode                            mode_;
  unsigned                        face_counter_;

  scm::gl::rasterizer_state_ptr   rs_cull_back_;
  scm::gl::rasterizer_state_ptr   rs_cull_none_;

  SubstitutionMap                 global_substitution_map_;
};

}

#endif  // GUA_DEPTH_CUBEMAP_RENDERER_HPP
