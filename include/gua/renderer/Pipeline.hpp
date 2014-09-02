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
#include <gua/utils/configuration_macro.hpp>
#include <gua/math.hpp>

#include <memory>
#include <list>

namespace gua {

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
    GUA_ADD_PROPERTY(math::vec2ui, left_resolution, math::vec2ui(800, 600));
    GUA_ADD_PROPERTY(math::vec2ui, right_resolution, math::vec2ui(800, 600));

    // // various display options
    // GUA_ADD_PROPERTY(bool, enable_preview_display, false);
    // GUA_ADD_PROPERTY(bool, enable_fps_display, false);
    // GUA_ADD_PROPERTY(bool, enable_ray_display, false);
    // GUA_ADD_PROPERTY(bool, enable_bbox_display, false);
    // GUA_ADD_PROPERTY(bool, enable_wireframe, false);
    // // FXAA
    // GUA_ADD_PROPERTY(bool, enable_fxaa, false);
    // // clipping
    // GUA_ADD_PROPERTY(float, near_clip, 0.1f);
    // GUA_ADD_PROPERTY(float, far_clip, 1000.0f);
    // // culling
    // GUA_ADD_PROPERTY(bool, enable_frustum_culling, true);
    // GUA_ADD_PROPERTY(bool, enable_backface_culling, true);
    // // screen space ambient occlusion
    // GUA_ADD_PROPERTY(bool, enable_ssao, false);
    // GUA_ADD_PROPERTY(float, ssao_radius, 2.0f);
    // GUA_ADD_PROPERTY(float, ssao_intensity, 1.0f);
    // GUA_ADD_PROPERTY(float, ssao_falloff, 1.0f);
    // // bloom
    // GUA_ADD_PROPERTY(bool, enable_bloom, false);
    // GUA_ADD_PROPERTY(float, bloom_radius, 10.0f);
    // GUA_ADD_PROPERTY(float, bloom_threshold, 0.8f);
    // GUA_ADD_PROPERTY(float, bloom_intensity, 0.4f);
    // // fog
    // GUA_ADD_PROPERTY(bool, enable_fog, false);
    // GUA_ADD_PROPERTY(float, fog_start, 100.0f);
    // GUA_ADD_PROPERTY(float, fog_end, 1000.0f);
    // GUA_ADD_PROPERTY(std::string, fog_texture, "");
    // GUA_ADD_PROPERTY(utils::Color3f, fog_color, utils::Color3f());
    // // background image / color
    // GUA_ADD_PROPERTY(BackgroundMode, background_mode, BackgroundMode::COLOR);
    // GUA_ADD_PROPERTY(std::string, background_texture, "");
    // GUA_ADD_PROPERTY(utils::Color3f, background_color, utils::Color3f());
    // // ambient color
    // GUA_ADD_PROPERTY(utils::Color3f, ambient_color, utils::Color3f(0.1f, 0.1f, 0.1f));
    // // vignette
    // GUA_ADD_PROPERTY(bool, enable_vignette, false);
    // GUA_ADD_PROPERTY(utils::Color3f, vignette_color, utils::Color3f());
    // GUA_ADD_PROPERTY(float, vignette_coverage, 0.3f);
    // GUA_ADD_PROPERTY(float, vignette_softness, 0.5f);
    // // HDR
    // GUA_ADD_PROPERTY(bool, enable_hdr, false);
    // GUA_ADD_PROPERTY(float, hdr_key, 1.f);
    // // NURBS tesselation
    // GUA_ADD_PROPERTY(int, max_tesselation, 4);
    // GUA_ADD_PROPERTY(float, tesselation_max_error, 8.0f);
  };

  ~Pipeline() {
    for (auto pass: passes_) {
      delete pass;
    }
  }

  Configuration config;

  template<class T>
  T& add_pass() {
    T* t = new T();
    passes_.push_back(t);
    return *t;
  }

  void set_output_texture_name(std::string const& name) {
    output_texture_name_ = name;
  }

  std::list<PipelinePass*> const& get_passes() const {
    return passes_;
  }

  void process(std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
               float application_fps, float rendering_fps) {
    for (auto pass: passes_) {
      pass->process();
    }
  }

 private:
  std::list<PipelinePass*> passes_;
  std::string output_texture_name_;
};

}

#endif  // GUA_PIPELINE_HPP
