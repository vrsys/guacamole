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

// guacamole headers
#include <gua/renderer/Camera.hpp>
#include <gua/renderer/Window.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/utils/Color3f.hpp>
#include <gua/utils/configuration_macro.hpp>

// external headers
#include <map>

namespace gua {

class Pass;
class SceneGraph;
class LayerMapping;
class StereoBuffer;
class GBufferMeshUberShader;
class LightingUberShader;
class FinalUberShader;
class PostFXShader;
class Serializer;

struct PipelineConfiguration {

  // camera for this pipeline
  GUA_ADD_PROPERTY(Camera, camera, Camera());

  // the final image of this pipeline will be stored in the texture database
  // with this name. if enable_stereo is set to true, two images with postfixes
  // _left and _right will be stored
  GUA_ADD_PROPERTY(std::string, output_texture_name, "gua_pipeline");

  // stereo configuration
  GUA_ADD_PROPERTY(math::vec2ui, left_resolution, math::vec2ui(800, 600));
  GUA_ADD_PROPERTY(math::vec2ui, right_resolution, math::vec2ui(800, 600));
  GUA_ADD_PROPERTY(bool, enable_stereo, false);

  // various display options
  GUA_ADD_PROPERTY(bool, enable_preview_display, false);
  GUA_ADD_PROPERTY(bool, enable_fps_display, false);
  GUA_ADD_PROPERTY(bool, enable_ray_display, false);
  GUA_ADD_PROPERTY(bool, enable_bbox_display, false);
  GUA_ADD_PROPERTY(bool, enable_wireframe, false);

  // FXAA
  GUA_ADD_PROPERTY(bool, enable_fxaa, false);

  // clipping
  GUA_ADD_PROPERTY(float, near_clip, 0.1f);
  GUA_ADD_PROPERTY(float, far_clip, 1000.0f);

  // culling
  GUA_ADD_PROPERTY(bool, enable_frustum_culling, true);
  GUA_ADD_PROPERTY(bool, enable_backface_culling, true);

  // screen space ambient occlusion
  GUA_ADD_PROPERTY(bool, enable_ssao, false);
  GUA_ADD_PROPERTY(float, ssao_radius, 2.0f);
  GUA_ADD_PROPERTY(float, ssao_intensity, 1.0f);
  GUA_ADD_PROPERTY(float, ssao_falloff, 1.0f);

  // bloom
  GUA_ADD_PROPERTY(bool, enable_bloom, false);
  GUA_ADD_PROPERTY(float, bloom_radius, 10.0f);
  GUA_ADD_PROPERTY(float, bloom_threshold, 0.8f);
  GUA_ADD_PROPERTY(float, bloom_intensity, 0.4f);

  // fog
  GUA_ADD_PROPERTY(bool, enable_fog, false);
  GUA_ADD_PROPERTY(float, fog_start, 100.0f);
  GUA_ADD_PROPERTY(float, fog_end, 1000.0f);
  GUA_ADD_PROPERTY(std::string, fog_texture, "");
  GUA_ADD_PROPERTY(utils::Color3f, fog_color, utils::Color3f());

  // background image / color
  GUA_ADD_PROPERTY(std::string, background_texture, "");
  GUA_ADD_PROPERTY(utils::Color3f, background_color, utils::Color3f());

  // ambient color
  GUA_ADD_PROPERTY(utils::Color3f, ambient_color, utils::Color3f(0.1, 0.1, 0.1));

  // vignette
  GUA_ADD_PROPERTY(bool, enable_vignette, false);
  GUA_ADD_PROPERTY(utils::Color3f, vignette_color, utils::Color3f());
  GUA_ADD_PROPERTY(float, vignette_coverage, 0.3f);
  GUA_ADD_PROPERTY(float, vignette_softness, 0.5f);

  // HDR
  GUA_ADD_PROPERTY(bool, enable_hdr, false);
  GUA_ADD_PROPERTY(float, hdr_key, 1.f);

  // NURBS tesselation
  GUA_ADD_PROPERTY(int, max_tesselation, 4);
  GUA_ADD_PROPERTY(float, tesselation_max_error, 8.0f);
};

/**
 * A rendering pipeline describes how an image is generated.
 *
 * A rendering pipeline consists of multiple passes. These are rendering parts
 * of (or the entire) SceneGraph to buffers. These buffers may be used as input
 * for other passes. One final buffer of a final pass is shown on the screen.
 */
class Pipeline {
 public:

  /**
   * Constructor.
   *
   * Creates a new Pipeline.
   */
  Pipeline();

  /**
   * Destructor.
   *
   * Deletes the Pipeline and frees all associated data.
   */
  virtual ~Pipeline();

  PipelineConfiguration config;

  void print_shaders(std::string const& directory) const;

  void set_window(Window* window);
  Window const* get_window() const;

  void set_prerender_pipelines(std::vector<Pipeline*> const& pipes);
  std::vector<Pipeline*> const& get_prerender_pipelines() const;

  float get_application_fps() const;
  float get_rendering_fps() const;

  friend class Renderer;
  friend class GBufferPass;
  friend class LightingPass;
  friend class FinalPass;
  friend class PostFXPass;
  friend class GeometryPass;
  friend class FullscreenPass;

 private:
  void process(std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
               float application_fps, float rendering_fps);

  void set_context(RenderContext* ctx);
  void create_passes();
  void create_buffers();

  SerializedScene const& get_current_scene(CameraMode mode) const;
  SceneGraph const* get_current_graph() const;

#if GUA_COMPILER == GUA_COMPILER_MSVC&& GUA_COMPILER_VER <= 1600
  mutable boost::mutex upload_mutex_;
#else
  mutable std::mutex upload_mutex_;
#endif

  Window* window_;
  RenderContext* context_;

  std::vector<Pipeline*> prerender_pipelines_;
  std::vector<Pass*> passes_;

  const SceneGraph* current_graph_;

  Serializer* serializer_;
  std::vector<SerializedScene> current_scenes_;

  float application_fps_;
  float rendering_fps_;

  bool passes_need_reload_;
  bool buffers_need_reload_;

  unsigned last_shading_model_revision_;
};

}

#endif  // GUA_PIPELINE_HPP
