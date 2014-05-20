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

#ifndef GUA_PASS_HPP
#define GUA_PASS_HPP

// guacamole headers
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/FrameBufferObject.hpp>
#include <gua/renderer/TextRenderer.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/enums.hpp>
#include <gua/utils/Mask.hpp>

// external headers
#include <memory>
#include <string>
#include <map>

namespace gua {

struct Camera;
struct SerializedScene;
class LayerMapping;
class Pipeline;
class SceneGraph;

/**
 * A database for accessing data.
 *
 * A virtual base class for render passes. It has a name, manages addition of
 * buffers and provides a basic interface which has to be implemented by
 * derived classes.
 *
 * Render passes are part of a rendering pipeline. Basically they encapsulate
 * some FBOs to which the scene is rendered. The user has to add some color
 * buffers to this pass and a depth stencil buffer if desired. The scene is
 * rendered frome the point of view of a given camera through a given screen.
 * With render masks a part of the scene may be hidden.
 */
class Pass {
 public:

  /**
   *
   */
  Pass(Pipeline* pipeline);

  virtual void render_scene(Camera const& camera,
                            SceneGraph const& current_graph,
                            RenderContext const& ctx) = 0;

  // not strictly necessary to call, but recommend
  // to avoid crashes on shader compilation
  // derived classes should call upload_to() on all contained shaders
  virtual bool pre_compile_shaders(RenderContext const& ctx) = 0;

  virtual void create(
      RenderContext const& ctx,
      std::vector<std::pair<BufferComponent,
                            scm::gl::sampler_state_desc> > const& layers);

  virtual void cleanup(RenderContext const& ctx);

  void set_inputs(std::vector<std::shared_ptr<StereoBuffer>> inputs);

  inline std::shared_ptr<StereoBuffer> get_gbuffer() const { return gbuffer_; }

  virtual LayerMapping const* get_gbuffer_mapping() const = 0;

 protected:
  void bind_inputs(ShaderProgram const& shader,
                   CameraMode eye,
                   RenderContext const& ctx) const;

  void set_camera_matrices(ShaderProgram const& shader,
                           Camera const& camera,
                           SerializedScene const& scene,
                           CameraMode eye,
                           RenderContext const& ctx) const;

  Pipeline* pipeline_;
  std::shared_ptr<StereoBuffer> gbuffer_;

  std::vector<std::shared_ptr<StereoBuffer>> inputs_;
  bool initialized_;
};

}

#endif  // GUA_PASS_HPP
