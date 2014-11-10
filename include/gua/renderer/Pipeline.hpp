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

#ifndef GUA_PIPELINE_HPP
#define GUA_PIPELINE_HPP

#include <gua/node/CameraNode.hpp>
#include <gua/renderer/Renderer.hpp>
#include <gua/renderer/PipelinePass.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/math.hpp>

#include <scm/gl_util/primitives/quad.h>

#include <memory>

namespace gua {

class GBuffer;
class WindowBase;
class RenderContext;
class ShaderProgram;

class Pipeline {
 public:

  Pipeline();
  ~Pipeline();

  void process(RenderContext* ctx, node::SerializedCameraNode const& camera,
               std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
               float application_fps, float rendering_fps);

  std::vector<PipelinePass*>  const& get_passes()  const;
  GBuffer                          & get_gbuffer() const;
  SerializedScene                  & get_scene();
  SceneGraph                  const& get_graph()   const;
  RenderContext               const& get_context() const;
  node::SerializedCameraNode  const& get_camera()  const;

  void bind_gbuffer_input(std::shared_ptr<ShaderProgram> const& shader) const;
  void bind_camera_uniform_block(unsigned location) const;
  void draw_fullscreen_quad();

 private:

  int                                fps_count_;
  math::vec2                         fps_sum_;

  GBuffer*                           gbuffer_;
  RenderContext*                     context_;
  CameraUniformBlock*                camera_block_;

  SceneGraph const*                  current_graph_;
  SerializedScene                    current_scene_;
  node::SerializedCameraNode         current_camera_;

  math::vec2ui                       last_resolution_;
  PipelineDescription                last_description_;

  std::vector<PipelinePass*>         passes_;
  scm::gl::quad_geometry_ptr         fullscreen_quad_;

};

}

#endif  // GUA_PIPELINE_HPP
