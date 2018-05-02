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

#ifndef GUA_SPOINTS_RENDERER_HPP
#define GUA_SPOINTS_RENDERER_HPP

#include <array>

// guacamole headers
#include <gua/spoints/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <unordered_map>

namespace gua {

class ShaderProgram;
class Pipeline;
class MaterialShader;
class SPointsResource;

class GUA_SPOINTS_DLL SPointsRenderer {
 public:

  enum pass {
    warp_pass = 0,
    blend_pass = 1
  };

 public:

  SPointsRenderer();
  ~SPointsRenderer() {std::cout << "SPOINTSRENDERER DESTROYED\n";}

  void render(Pipeline& pipe, PipelinePassDescription const& desc);

  // /*virtual*/ void draw   (RenderContext const& context,
  //                          std::string const& ksfile_name,
  //                          std::string const& material_name,
  //                          scm::math::mat4 const& model_matrix,
  //                          scm::math::mat4 const& normal_matrix,
  //                          Frustum const& frustum,
  //                          View const& view) const;

  void set_global_substitution_map(SubstitutionMap const& smap) {
    global_substitution_map_ = smap;
  }

 private:  // attributes

  bool initialized_;

  std::vector<ShaderProgramStage> program_stages_;
  std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram> >
      programs_;
  SubstitutionMap global_substitution_map_;



  scm::gl::rasterizer_state_ptr points_rasterizer_state_;

};

}

#endif  // GUA_SPOINTS_RENDERER_HPP
