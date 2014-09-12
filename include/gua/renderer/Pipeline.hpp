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

#include <gua/renderer/Renderer.hpp>
#include <gua/renderer/PipelinePass.hpp>
#include <gua/renderer/Camera.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/FrameBufferObject.hpp>
#include <gua/renderer/RessourceRenderer.hpp>
#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/configuration_macro.hpp>
#include <gua/math.hpp>

#include <memory>
#include <list>

namespace gua {

class GBuffer;
class WindowBase;
class RenderContext;

class Pipeline {
 public:

  struct Configuration {

    // camera for this pipeline
    GUA_ADD_PROPERTY(Camera, camera, Camera());

    // if set to false, this pipeline won't render anything
    GUA_ADD_PROPERTY(bool, enabled, true);

    GUA_ADD_PROPERTY(bool, enable_stereo, false);

    // the final image of this pipeline will be stored in the texture database
    // with this name. if enable_stereo is set to true, two images with postfixes
    // _left and _right will be stored
    GUA_ADD_PROPERTY(std::string, output_texture_name, "gua_pipeline");

    // stereo configuration
    GUA_ADD_PROPERTY(math::vec2ui, resolution, math::vec2ui(800, 600));

    // various display options
    GUA_ADD_PROPERTY(bool, enable_ray_display, false);
    GUA_ADD_PROPERTY(bool, enable_bbox_display, false);

    // clipping
    GUA_ADD_PROPERTY(float, near_clip, 0.1f);
    GUA_ADD_PROPERTY(float, far_clip, 1000.0f);

    // culling
    GUA_ADD_PROPERTY(bool, enable_frustum_culling, true);
  };

  Pipeline();
  ~Pipeline();

  Configuration config;

  template<class T>
  T& add_pass() {
    dirty_ = true;
    T* t = new T();
    passes_.push_back(t);
    return *t;
  }

  void set_output_window(WindowBase* window);

  void process(std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
               float application_fps, float rendering_fps);

  std::list<PipelinePass*> const& get_passes()  const;
  GBuffer                       & get_gbuffer() const;
  RenderContext            const& get_context() const;
  SerializedScene          const& get_scene()   const;
  
  void bind_gbuffer_input(std::shared_ptr<ShaderProgram> const& shader) const;
  void bind_camera_uniform_block(unsigned location) const;
  void draw_fullscreen_quad();

  std::shared_ptr<RessourceRenderer> get_renderer(GeometryResource const& type);

 private:
  std::list<PipelinePass*> passes_;
  GBuffer*                 gbuffer_;
  WindowBase*              window_;
  SerializedScene          current_scene_;
  CameraUniformBlock*      camera_block_;

  bool                     dirty_;
  std::unordered_map<std::type_index, std::shared_ptr<RessourceRenderer>> renderers_; 

  scm::gl::quad_geometry_ptr fullscreen_quad_;

};

}

#endif  // GUA_PIPELINE_HPP
