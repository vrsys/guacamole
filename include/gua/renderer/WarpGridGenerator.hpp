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

#ifndef GUA_WARP_GENERATOR_HPP
#define GUA_WARP_GENERATOR_HPP

#include <map>
#include <unordered_map>

#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <scm/gl_core/shader_objects.h>

namespace gua {

class Pipeline;
class PipelinePassDescription;

class WarpGridGenerator {

 public:

  struct SharedResource {
    inline int current_tfb() const { return ping ? 1 : 0; }
    inline int current_vbo() const { return ping ? 0 : 1; }

    bool ping = false;

    scm::gl::transform_feedback_ptr grid_tfb[2];
    scm::gl::buffer_ptr             grid_vbo[2];
    scm::gl::vertex_array_ptr       grid_vao[2];
    size_t                          cell_count = 0;

    std::shared_ptr<Texture>                surface_detection_buffer;
    std::vector<scm::gl::frame_buffer_ptr>  surface_detection_buffer_fbos;
  };

  WarpGridGenerator();
  virtual ~WarpGridGenerator();

  void render(Pipeline& pipe, PipelinePassDescription const& desc);
  void set_global_substitution_map(SubstitutionMap const& smap) { global_substitution_map_ = smap; }

 private:
  std::shared_ptr<SharedResource>  res_;
  SubstitutionMap                  global_substitution_map_;

  std::vector<ShaderProgramStage>  grid_generation_program_stages_;
  std::shared_ptr<ShaderProgram>   grid_generation_program_;

  std::vector<ShaderProgramStage>  surface_detection_program_stages_;
  std::shared_ptr<ShaderProgram>   surface_detection_program_;

  Pipeline* pipe_;
};

}

#endif  // GUA_WARP_GENERATOR_HPP
