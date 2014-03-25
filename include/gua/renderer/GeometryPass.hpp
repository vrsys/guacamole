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

#ifndef GUA_GEOMETRY_PASS_HPP
#define GUA_GEOMETRY_PASS_HPP

// guacamole headers
#include <gua/renderer/Pass.hpp>

// external headers
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

namespace gua {

class ShaderProgram;

/**
 * A render pass which draws a part of the SceneGraph.
 *
 * This render pass is the most commonly used pass in rendering pipelines.
 *
 * Render passes are part of a rendering pipeline. Basically they encapsulate
 * some FBOs to which the scene is rendered. The user has to add some color
 * buffers to this pass and a depth stencil buffer if desired. The scene is
 * rendered frome the point of view of a given camera through a given screen.
 * With render masks a part of the scene may be hidden.
 */
class GeometryPass : public Pass {
 public:

  /**
   *
   */
  GeometryPass(Pipeline* pipeline);

  /**
   * Destructor.
   *
   * Deletes the GeometryPass and frees all associated data.
   */
  virtual ~GeometryPass() {}

  virtual void render_scene(Camera const& camera,
                            SceneGraph const* current_graph,
                            RenderContext const& ctx);

 protected:
  virtual void rendering(SerializedScene const& scene,
                         SceneGraph const* scene_graph,
                         RenderContext const& ctx,
                         CameraMode eye,
                         Camera const& camera,
                         FrameBufferObject* target) = 0;

 private:
};

}

#endif  // GUA_GEOMETRY_PASS_HPP
