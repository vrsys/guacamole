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

#ifndef GUA_FULLSCREEN_PASS_HPP
#define GUA_FULLSCREEN_PASS_HPP

// guacamole headers
#include <gua/renderer/Pass.hpp>

// external headers
#include <scm/gl_util/primitives/quad.h>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

namespace gua {

class ShaderProgram;
class SceneGraph;

/**
 * A GeometryPass which automatically renders to a fullscreen quad.
 *
 * This is especially useful for deferred shading or other post
 * processing purposes.
 *
 * Render passes are part of a rendering pipeline. Basically they encapsulate
 * some FBOs to which the scene is rendered. The user has to add some color
 * buffers to this pass and a depth stencil buffer if desired. The scene is
 * rendered frome the point of view of a given camera through a given screen.
 * With render masks a part of the scene may be hidden.
 */
class FullscreenPass : public Pass {
 public:

  /**
   *
   */
  FullscreenPass(Pipeline* pipeline);

  /**
   * Destructor.
   *
   * Deletes the FullscreenPass and frees all associated data.
   */
  virtual ~FullscreenPass();

  /**
   *
   */
  void render_scene(Camera const& camera, SceneGraph const&, RenderContext const& ctx, std::size_t viewid);

 protected:

  /**
   *
   */
  virtual void set_uniforms(SerializedScene const& scene,
                            RenderContext const& ctx) {}

  /**
   *
   */
  virtual void pre_rendering(Camera const& camera,
                             SerializedScene const& scene,
                             CameraMode eye,
                             RenderContext const& ctx) {}

  /**
   *
   */
  virtual void rendering(Camera const& camera,
                         SerializedScene const& scene,
                         CameraMode eye,
                         RenderContext const& ctx) = 0;

  /**
   *
   */
  virtual void post_rendering(Camera const& camera,
                              SerializedScene const& scene,
                              CameraMode eye,
                              RenderContext const& ctx) {}

  scm::gl::quad_geometry_ptr fullscreen_quad_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_;

 private:
};

}

#endif  // GUA_FULLSCREEN_PASS_HPP
