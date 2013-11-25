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

#ifndef GUA_COMPOSITE_PASS_HPP
#define GUA_COMPOSITE_PASS_HPP

// guacamole headers
#include <gua/renderer/BuiltInTextures.hpp>
#include <gua/renderer/Pass.hpp>

namespace gua {

class GBuffer;
struct PipelineConfiguration;

/**
 *
 */
class CompositePass : public Pass {
 public:

  /**
   *
   */
	 CompositePass(Pipeline* pipeline);

  /**
   * 
   */
	virtual ~CompositePass();

  void create(
      RenderContext const& ctx,
      PipelineConfiguration const& config,
      std::vector<std::pair<BufferComponent,
                            scm::gl::sampler_state_desc> > const& layers);

  void render_scene(Camera const& camera, RenderContext const& ctx);

  /* virtual */ LayerMapping const* get_gbuffer_mapping() const;

  void print_shaders(std::string const& directory,
                     std::string const& name) const;

  bool pre_compile_shaders(RenderContext const& ctx);

 private:

  void apply_material_mapping();

  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
  scm::gl::quad_geometry_ptr fullscreen_quad_;
  ShaderProgram* composite_shader_;
};

}

#endif  // GUA_COMPOSITE_PASS_HPP
