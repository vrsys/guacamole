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

#ifndef GUA_WARP_RENDERER_HPP
#define GUA_WARP_RENDERER_HPP

#include <map>
#include <unordered_map>

#include <gua/renderer/WarpPass.hpp>
#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <scm/gl_core/shader_objects.h>

namespace gua {

class Pipeline;
class PipelinePassDescription;

class WarpRenderer {

 public:

  WarpRenderer();
  virtual ~WarpRenderer() {}

  void render(Pipeline& pipe, PipelinePassDescription const& desc);

  void set_global_substitution_map(SubstitutionMap const& smap) { global_substitution_map_ = smap; }

 private:

  WarpPassDescription::WarpState   cached_warp_state_;

  scm::gl::vertex_array_ptr        empty_vao_;
  scm::gl::rasterizer_state_ptr    points_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_yes_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_no_;
  SubstitutionMap                  global_substitution_map_;

  std::vector<ShaderProgramStage>  warp_gbuffer_program_stages_;
  std::shared_ptr<ShaderProgram>   warp_gbuffer_program_;

  std::vector<ShaderProgramStage>  warp_abuffer_program_stages_;
  std::shared_ptr<ShaderProgram>   warp_abuffer_program_;
};

}

#endif  // GUA_WARP_RENDERER_HPP
