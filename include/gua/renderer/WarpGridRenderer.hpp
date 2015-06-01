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

#ifndef GUA_WARP_GRID_RENDERER_HPP
#define GUA_WARP_GRID_RENDERER_HPP

#include <map>
#include <unordered_map>

#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <scm/gl_core/shader_objects.h>

namespace gua {

class Pipeline;
class PipelinePassDescription;

class WarpGridRenderer {

 public:

  WarpGridRenderer();
  virtual ~WarpGridRenderer() {}

  void render(Pipeline& pipe, PipelinePassDescription const& desc);

 private:
  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
  scm::gl::rasterizer_state_ptr    rasterizer_state_;
  scm::gl::blend_state_ptr         blend_state_;
  std::vector<ShaderProgramStage>  program_stages_;
  std::shared_ptr<ShaderProgram>   program_;
};

}

#endif  // GUA_WARP_GRID_RENDERER_HPP
