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
#include <gua/platform.hpp>
#include <gua/renderer/Camera.hpp>
#include <gua/renderer/Window.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/renderer/GBufferPass.hpp>
#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/utils/Color3f.hpp>
#include <gua/utils/configuration_macro.hpp>

// external headers
#include <map>

namespace gua {

class Pass;
class SceneGraph;
class LayerMapping;
class StereoBuffer;
class TriMeshUberShader;
class LightingUberShader;
class FinalUberShader;
class PostFXShader;
class Serializer;

/**
 * A rendering pipeline describes how an image is generated.
 *
 * A rendering pipeline consists of multiple passes. These are rendering parts
 * of (or the entire) SceneGraph to buffers. These buffers may be used as input
 * for other passes. One final buffer of a final pass is shown on the screen.
 */
class GUA_DLL Pipeline {
 public:

  enum BackgroundMode {
    COLOR = 0,
    SKYMAP_TEXTURE = 1,
    QUAD_TEXTURE = 2,
  };

  struct Configuration {

    // camera for this pipeline
    GUA_ADD_PROPERTY(Camera, camera, Camera());

    // if set to false, this pipeline won't render anything
    GUA_ADD_PROPERTY(bool, enabled, true);

    // global clipping plane nothing
    GUA_ADD_PROPERTY(bool, enable_global_clipping_plane, false);
    GUA_ADD_PROPERTY(math::vec4, global_clipping_plane, math::vec4(0, 1, 0, 0));

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
    GUA_ADD_PROPERTY(BackgroundMode, background_mode, BackgroundMode::COLOR);
    GUA_ADD_PROPERTY(std::string, background_texture, "");
    GUA_ADD_PROPERTY(utils::Color3f, background_color, utils::Color3f());

    // ambient color
    GUA_ADD_PROPERTY(utils::Color3f, ambient_color, utils::Color3f(0.1f, 0.1f, 0.1f));

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

  enum PipelineStage {
    geometry = 0,
    lighting = 1,
    shading = 2,
    compositing = 3,
    postfx = 4
  };

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

  Configuration config;

  void print_shaders(std::string const& directory) const;

  void set_window(Window* window);
  inline Window const* get_window() const { return window_; }

  void set_prerender_pipelines(std::vector<Pipeline*> const& pipes);
  std::vector<Pipeline*> const& get_prerender_pipelines() const {
    return prerender_pipelines_;
  }

  inline float get_application_fps() const { return application_fps_; }
  inline float get_rendering_fps() const { return rendering_fps_; }
  inline RenderContext const& get_context() const { return *context_; };

  SerializedScene const& get_current_scene(CameraMode mode) const;

  inline std::size_t uuid() const {
    return reinterpret_cast<std::size_t>(this);
  }
  GBufferPass::GeometryUberShaderMap const& get_geometry_ubershaders() const;

  friend class Renderer;
  friend class Pass;

 private:
  void loading_screen();
  void serialize(const SceneGraph& scene_graph,
                 std::string const& eye_name,
                 std::string const& screen_name,
                 SerializedScene& out);

  void process(std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
               float application_fps, float rendering_fps);

  bool validate_resolution() const;
  void set_context(RenderContext* ctx);
  void create_passes();
  void create_buffers();

  mutable std::mutex upload_mutex_;

  Window* window_;
  RenderContext* context_;

  std::vector<Pipeline*> prerender_pipelines_;
  std::vector<Pass*> passes_;

  Serializer* serializer_;
  std::vector<SerializedScene> current_scenes_;

  float application_fps_;
  float rendering_fps_;

  bool passes_need_reload_;
  bool buffers_need_reload_;
  bool display_loading_screen_;

  unsigned last_shading_model_revision_;

  std::shared_ptr<gua::CameraUniformBlock> camera_block_;
};

}

#endif  // GUA_PIPELINE_HPP
