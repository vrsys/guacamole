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

// class header
#include <gua/renderer/Pipeline.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GBufferMeshUberShader.hpp>
#include <gua/renderer/LightingUberShader.hpp>
#include <gua/renderer/FinalUberShader.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/renderer/GBufferPass.hpp>
#include <gua/renderer/LightingPass.hpp>
#include <gua/renderer/FinalPass.hpp>
#include <gua/renderer/PostFXPass.hpp>
#include <gua/renderer/Serializer.hpp>
#include <gua/scenegraph.hpp>
#include <gua/utils/logger.hpp>
#include <gua/databases.hpp>

// external headers
#include <iostream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Pipeline::Pipeline()
    : config(),
      window_(nullptr),
      context_(nullptr),
      current_graph_(nullptr),
      application_fps_(0),
      rendering_fps_(0),
      serializer_(new Serializer()),
      current_scenes_(2),
      passes_need_reload_(true),
      buffers_need_reload_(true),
      last_shading_model_revision_(0) {

        create_passes();
      }

////////////////////////////////////////////////////////////////////////////////

Pipeline::~Pipeline() {
  if (serializer_)
    delete serializer_;

  if (window_)
    delete window_;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::print_shaders(std::string const& directory) const {

  std::unique_lock<std::mutex> lock(upload_mutex_);

  passes_[0]->print_shaders(directory, "/0_gbuffer");
  passes_[1]->print_shaders(directory, "/1_lighting");
  passes_[2]->print_shaders(directory, "/2_final");
  passes_[3]->print_shaders(directory, "/3_postFX");
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::set_window(Window* window) {
  std::unique_lock<std::mutex> lock(upload_mutex_);
  window_ = window;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::set_prerender_pipelines(std::vector<Pipeline*> const& pipes) {
  std::unique_lock<std::mutex> lock(upload_mutex_);
  prerender_pipelines_ = pipes;
}

////////////////////////////////////////////////////////////////////////////////

SerializedScene const& Pipeline::get_current_scene(CameraMode mode) const {
  if (mode == CameraMode::RIGHT && current_scenes_.size() > 1) {
    return current_scenes_[1];
  }
  return current_scenes_[0];
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::process(std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
                       float application_fps,
                       float rendering_fps) {

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (ShadingModel::current_revision != last_shading_model_revision_) {
    passes_need_reload_ = true;
    last_shading_model_revision_ = ShadingModel::current_revision;
  }

  if (window_) {
    if (!window_->get_is_open()) {
      window_->open();
      window_->create_shader();
    }

    set_context(window_->get_context());
    window_->set_active(true);
  }


  for (auto pipe: prerender_pipelines_) {
    pipe->process(scene_graphs, application_fps, rendering_fps);
  }

  current_graph_ = nullptr;

  for (auto& graph: scene_graphs) {
    if (graph->get_name() == config.camera().scene_graph) {
      current_graph_ = graph.get();
      break;
    }
  }

  if (!current_graph_) {
    WARNING("Failed to display scenegraph \"%s\": Graph not found!",
            config.camera().scene_graph.c_str());
  }

  rendering_fps_ = rendering_fps;
  application_fps_ = application_fps;

  if (!context_) {
    WARNING("Pipeline has no context: Please define a window!");
    return;
  }

  if (passes_need_reload_) {
    create_passes();
  }

  if (buffers_need_reload_) {
    create_buffers();
  }





  if (!config.get_enable_stereo()) {

    auto eye((*current_graph_)[config.camera().eye_l]);
    if (!eye) {
      WARNING("Cannot render scene: No valid eye specified");
      return;
    }

    auto screen_it((*current_graph_)[config.camera().screen_l]);
    auto screen(std::dynamic_pointer_cast<ScreenNode>(screen_it));
    if (!screen) {
      WARNING("Cannot render scene: No valid screen specified");
      return;
    }

    current_scenes_[0].frustum = Frustum(eye->get_world_transform(),
                                         screen->get_scaled_world_transform(),
                                         config.near_clip(),
                                         config.far_clip());

    serializer_->check(&current_scenes_[0],
                       current_graph_,
                       config.camera(),
                       config.enable_bbox_display(),
                       config.enable_ray_display(),
                       config.enable_frustum_culling());
  } else {


    auto eye_l((*current_graph_)[config.camera().eye_l]);
    if (!eye_l) {
      WARNING("Cannot render scene: No valid left eye specified");
      return;
    }

    auto eye_r((*current_graph_)[config.camera().eye_r]);
    if (!eye_r) {
      WARNING("Cannot render scene: No valid right eye specified");
      return;
    }

    auto screen_it_l((*current_graph_)[config.camera().screen_l]);
    auto screen_l(std::dynamic_pointer_cast<ScreenNode>(screen_it_l));
    if (!screen_l) {
      WARNING("Cannot render scene: No valid left screen specified");
      return;
    }

    auto screen_it_r((*current_graph_)[config.camera().screen_r]);
    auto screen_r(std::dynamic_pointer_cast<ScreenNode>(screen_it_r));
    if (!screen_r) {
      WARNING("Cannot render scene: No valid right screen specified");
      return;
    }


    current_scenes_[0].frustum = Frustum(eye_l->get_world_transform(),
                                         screen_l->get_scaled_world_transform(),
                                         config.near_clip(),
                                         config.far_clip());
    current_scenes_[1].frustum = Frustum(eye_r->get_world_transform(),
                                         screen_r->get_scaled_world_transform(),
                                         config.near_clip(),
                                         config.far_clip());

    serializer_->check(&current_scenes_[0],
                       current_graph_,
                       config.camera(),
                       config.enable_bbox_display(),
                       config.enable_ray_display(),
                       config.enable_frustum_culling());

    serializer_->check(&current_scenes_[1],
                       current_graph_,
                       config.camera(),
                       config.enable_bbox_display(),
                       config.enable_ray_display(),
                       config.enable_frustum_culling());
  }

  for (int i(0); i < passes_.size(); ++i) {
    passes_[i]->render_scene(config.camera(), *context_);
  }

  if (window_) {
    if (config.get_enable_stereo()) {
        window_->display(passes_[3]->get_gbuffer()->get_eye_buffers()[0]
                             ->get_color_buffers(TYPE_FLOAT)[0],
                         passes_[3]->get_gbuffer()->get_eye_buffers()[1]
                             ->get_color_buffers(TYPE_FLOAT)[0]);
    } else {
      window_->display(passes_[3]->get_gbuffer()->get_eye_buffers()[0]
                           ->get_color_buffers(TYPE_FLOAT)[0]);
    }

    window_->finish_frame();
  }
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::set_context(RenderContext* ctx) {
  context_ = ctx;

  for (auto pipe: prerender_pipelines_) {
    pipe->set_context(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::create_passes() {

  if (passes_need_reload_) {


    auto materials(MaterialDatabase::instance()->list_all());

    auto pre_pass = new GBufferPass(this);
    pre_pass->apply_material_mapping(materials);

    std::vector<LayerMapping const*> layer_mapping;
    layer_mapping.push_back(pre_pass->get_gbuffer_mapping());

    auto light_pass = new LightingPass(this);
    light_pass->apply_material_mapping(materials, layer_mapping);

    layer_mapping.push_back(light_pass->get_gbuffer_mapping());

    auto final_pass = new FinalPass(this);
    final_pass->apply_material_mapping(materials, layer_mapping);

    auto post_fx_pass = new PostFXPass(this);

    bool compilation_succeeded = false;

    passes_need_reload_ = false;

    // try compilation if context is already present
    if (context_) {

      if (pre_pass->pre_compile_shaders(*context_))
      if (light_pass->pre_compile_shaders(*context_))
      if (final_pass->pre_compile_shaders(*context_))
      if (post_fx_pass->pre_compile_shaders(*context_))

        compilation_succeeded = true;

    } else {
      compilation_succeeded = true;
    }

    if (compilation_succeeded) {

      for (int i(0); i < passes_.size(); ++i) {
        delete passes_[i];
      }

      passes_.clear();

      passes_.push_back(pre_pass);
      passes_.push_back(light_pass);
      passes_.push_back(final_pass);
      passes_.push_back(post_fx_pass);

      buffers_need_reload_ = true;

    } else {

      delete pre_pass;
      delete light_pass;
      delete final_pass;
      delete post_fx_pass;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::create_buffers() {

  if (buffers_need_reload_) {
    passes_[0]->create(*context_, config,
                       passes_[0]->get_gbuffer_mapping()->get_layers());

    passes_[1]->create(*context_, config,
                       passes_[1]->get_gbuffer_mapping()->get_layers());

    std::vector<std::shared_ptr<StereoBuffer>> stereobuffers;
    stereobuffers.push_back(passes_[0]->get_gbuffer());

    passes_[1]->set_inputs(stereobuffers);

    stereobuffers.push_back(passes_[1]->get_gbuffer());

    passes_[2]->create(*context_,
                       config,
                       passes_[2]->get_gbuffer_mapping()->get_layers());

    passes_[2]->set_inputs(stereobuffers);

    scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_LINEAR,
                                      scm::gl::WRAP_REPEAT,
                                      scm::gl::WRAP_REPEAT);

  #if GUA_COMPILER == GUA_COMPILER_MSVC&& SCM_COMPILER_VER <= 1700
    std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> >
        layer_desc;
    layer_desc.push_back(std::make_pair(BufferComponent::F3, state));
    passes_[3]->create(*context_, config, layer_desc);
  #else
    passes_[3]->create(*context_, config,
                       {
      { BufferComponent::F3, state }
    });
  #endif
    stereobuffers.push_back(passes_[2]->get_gbuffer());
    passes_[3]->set_inputs(stereobuffers);

    if (!config.get_enable_stereo()) {
      TextureDatabase::instance()->add(config.output_texture_name(), passes_[3]->get_gbuffer()->get_eye_buffers()[0]->get_color_buffers(TYPE_FLOAT)[0]);
    } else {
      TextureDatabase::instance()->add(config.output_texture_name() + "_left",  passes_[3]->get_gbuffer()->get_eye_buffers()[0]->get_color_buffers(TYPE_FLOAT)[0]);
      TextureDatabase::instance()->add(config.output_texture_name() + "_right", passes_[3]->get_gbuffer()->get_eye_buffers()[1]->get_color_buffers(TYPE_FLOAT)[0]);
    }


    buffers_need_reload_ = false;
  }
}

////////////////////////////////////////////////////////////////////////////////

}
