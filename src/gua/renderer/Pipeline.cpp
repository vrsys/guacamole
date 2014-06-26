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
#include <gua/renderer/TriMeshUberShader.hpp>
#include <gua/renderer/LightingUberShader.hpp>
#include <gua/renderer/FinalUberShader.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/renderer/GBufferPass.hpp>
#include <gua/renderer/CompositePass.hpp>
#include <gua/renderer/LightingPass.hpp>
#include <gua/renderer/FinalPass.hpp>
#include <gua/renderer/PostFXPass.hpp>
#include <gua/renderer/Serializer.hpp>
#include <gua/scenegraph.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases.hpp>

// external headers
#include <iostream>


namespace {

gua::Frustum camera_frustum(gua::Camera::ProjectionMode const& mode,
    gua::math::mat4 const& transf, gua::math::mat4 const& screen,
    float near, float far) {
  if (mode == gua::Camera::ProjectionMode::PERSPECTIVE) {
    return gua::Frustum::perspective(transf, screen, near, far);
  } else {
    return gua::Frustum::orthographic(transf, screen, near, far);
  }
}

}


namespace gua {

////////////////////////////////////////////////////////////////////////////////

Pipeline::Pipeline()
  : config(),
    window_(nullptr),
    context_(nullptr),
    application_fps_(0),
    rendering_fps_(0),
    serializer_(new Serializer()),
    current_scenes_(2),
    passes_need_reload_(true),
    buffers_need_reload_(true),
    last_shading_model_revision_(0),
    display_loading_screen_(true),
    last_left_resolution_(0, 0),
    last_right_resolution_(0, 0),
    camera_block_left_(nullptr),
    camera_block_right_(nullptr) {

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

  int ctr(0);
  for (const auto& pass : passes_) {
    pass->print_shaders(directory, "/" + std::to_string(ctr++) + "_" +
            string_utils::sanitize(
                string_utils::demangle_type_name(typeid(*pass).name())));
  }
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

void Pipeline::loading_screen() {
  display_loading_screen_ = false;

  if (window_) {
    auto loading_texture(std::dynamic_pointer_cast<Texture2D>(TextureDatabase::instance()->lookup("gua_loading_texture")));
    math::vec2ui loading_texture_size(loading_texture->width(), loading_texture->height());

    if (config.get_enable_stereo()) {

      auto tmp_left_resolution(window_->config.left_resolution());
      auto tmp_right_resolution(window_->config.right_resolution());

      auto tmp_left_position(window_->config.left_position());
      auto tmp_right_position(window_->config.right_position());

      window_->config.set_left_resolution(loading_texture_size);
      window_->config.set_left_position(tmp_left_position + (tmp_left_resolution - loading_texture_size)/2);

      window_->config.set_right_resolution(loading_texture_size);
      window_->config.set_right_position(tmp_right_position + (tmp_right_resolution - loading_texture_size)/2);

      window_->display(loading_texture, loading_texture);

      window_->config.set_left_position(tmp_left_position);
      window_->config.set_left_resolution(tmp_left_resolution);

      window_->config.set_right_position(tmp_right_position);
      window_->config.set_right_resolution(tmp_right_resolution);

    } else {

      auto tmp_left_resolution(window_->config.left_resolution());
      auto tmp_left_position(window_->config.left_position());


      window_->config.set_left_resolution(loading_texture_size);
      window_->config.set_left_position(tmp_left_position + 0.5*(tmp_left_resolution - loading_texture_size));

      window_->display(loading_texture);

      window_->config.set_left_position(tmp_left_position);
      window_->config.set_left_resolution(tmp_left_resolution);

    }

    window_->finish_frame();
  }
}

void Pipeline::serialize(const SceneGraph& scene_graph,
                         std::string const& eye_name,
                         std::string const& screen_name,
                         SerializedScene& out) {
  auto eye((scene_graph)[eye_name]);
  if (!eye) {
    Logger::LOG_WARNING << "Cannot render scene: No valid eye specified" << std::endl;
    return;
  }

  auto screen_it((scene_graph)[screen_name]);
  auto screen(std::dynamic_pointer_cast<node::ScreenNode>(screen_it));
  if (!screen) {
    Logger::LOG_WARNING << "Cannot render scene: No valid screen specified" << std::endl;
    return;
  }

  out.frustum = camera_frustum(config.camera().mode, eye->get_world_transform(),
                                screen->get_scaled_world_transform(),
                                config.near_clip(),
                                config.far_clip());
  out.center_of_interest = eye->get_world_position();
  out.enable_global_clipping_plane = config.get_enable_global_clipping_plane();
  out.global_clipping_plane = config.get_global_clipping_plane();

  serializer_->check(out,
                     scene_graph,
                     config.camera().render_mask,
                     config.enable_bbox_display(),
                     config.enable_ray_display(),
                     config.enable_frustum_culling());
}


void Pipeline::process(std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
                       float application_fps,
                       float rendering_fps) {

  if (!config.get_enabled()) {
    return;
  }

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (ShadingModel::current_revision != last_shading_model_revision_) {
    passes_need_reload_ = true;
    last_shading_model_revision_ = ShadingModel::current_revision;
  }

  if (config.left_resolution() != last_left_resolution_ ||
      config.right_resolution() != last_right_resolution_) {

    buffers_need_reload_ = true;

    last_left_resolution_ = config.left_resolution();
    last_right_resolution_ = config.right_resolution();
  }

  if (window_) {
    if (!window_->get_is_open()) {
      window_->open();
      window_->create_shader();
    }

    set_context(window_->get_context());
    window_->set_active(true);
  }

  if (!camera_block_left_) {
    camera_block_left_ = std::make_shared<CameraUniformBlock>(context_->render_device);
    camera_block_right_ = std::make_shared<CameraUniformBlock>(context_->render_device);
  }

  for (auto pipe: prerender_pipelines_) {
    pipe->process(scene_graphs, application_fps, rendering_fps);
  }

  SceneGraph const* current_graph = nullptr;

  for (auto& graph: scene_graphs) {
    if (graph->get_name() == config.camera().scene_graph) {
      current_graph = graph.get();
      break;
    }
  }

  if (!current_graph) {
    Logger::LOG_WARNING << "Failed to display scenegraph \""
                        << config.camera().scene_graph << "\": Graph not found!"
                        << std::endl;
  }

  rendering_fps_ = rendering_fps;
  application_fps_ = application_fps;

  if (!context_) {
    Logger::LOG_WARNING << "Pipeline has no context: Please define a window!"
                        << std::endl;
    return;
  }

  if (display_loading_screen_) {
    loading_screen();
  } else {
    if (passes_need_reload_) {
      create_passes();
      camera_block_left_ = std::make_shared<CameraUniformBlock>(context_->render_device);
      camera_block_right_ = std::make_shared<CameraUniformBlock>(context_->render_device);
    }

    if (buffers_need_reload_) {
      create_buffers();
    }

    if (passes_need_reload_ || buffers_need_reload_) {
      Logger::LOG_WARNING << "Pipeline::process() : Passes or buffers not created yet. Skipping frame." << std::endl;
      return;
    }

    serialize(*current_graph, config.camera().eye_l, config.camera().screen_l,
              current_scenes_[0]);

    camera_block_left_->update(context_->render_context
                              , current_scenes_[0].frustum);

    if (config.get_enable_stereo()) {
      serialize(*current_graph, config.camera().eye_r, config.camera().screen_r,
              current_scenes_[1]);
      camera_block_right_->update(context_->render_context
                                , current_scenes_[1].frustum);
    }

    for (auto pass : passes_) {
      pass->render_scene(config.camera(), *current_graph, *context_, uuid());
    }

    if (window_) {
      if (config.get_enable_stereo()) {
          window_->display(passes_[PipelineStage::postfx]
                                  ->get_gbuffer()
                                  ->get_eye_buffers()[0]
                                  ->get_color_buffers(TYPE_FLOAT)[0],
                           passes_[PipelineStage::postfx]
                                  ->get_gbuffer()
                                  ->get_eye_buffers()[1]
                                  ->get_color_buffers(TYPE_FLOAT)[0]);
      } else {
        window_->display(passes_[PipelineStage::postfx]
                                ->get_gbuffer()
                                ->get_eye_buffers()[0]
                                ->get_color_buffers(TYPE_FLOAT)[0],
                         passes_[PipelineStage::postfx]
                                ->get_gbuffer()
                                ->get_eye_buffers()[0]
                                ->get_color_buffers(TYPE_FLOAT)[0]);
      }

      window_->finish_frame();
      ++(window_->get_context()->framecount);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

bool Pipeline::validate_resolution() const
{
  // first, validate configuration
  if (!config.get_enable_stereo()) // mono
  {
    if (!(config.get_left_resolution()[0] > 0 && config.get_left_resolution()[1] > 0))
    {
      gua::Logger::LOG_WARNING << "Pipeline::validate_resolution() : Invalid resolution for pipeline : " << config.get_left_resolution() << std::endl;
      return false;
    }
  }
  else { // stereo
    if (!(config.get_left_resolution()[0] > 0 && config.get_left_resolution()[1] > 0 &&
      config.get_right_resolution()[0] > 0 && config.get_right_resolution()[1] > 0))
    {
      gua::Logger::LOG_WARNING << "Pipeline::validate_resolution() : Invalid resolution for pipeline" << config.get_left_resolution() << " and " << config.get_right_resolution() << std::endl;
      return false;
    }
  }
  return true;
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

    if (!validate_resolution()) {
      return;
    }

    std::unique_lock<std::mutex> lock (MaterialDatabase::instance()->update_lock);

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

    auto composite_pass = new CompositePass(this);

    auto post_fx_pass = new PostFXPass(this);

    bool compilation_succeeded = true;

    passes_need_reload_ = false;

    std::vector<Pass*> new_passes;
    new_passes.push_back(pre_pass);
    new_passes.push_back(light_pass);
    new_passes.push_back(final_pass);
    new_passes.push_back(composite_pass);
    new_passes.push_back(post_fx_pass);

    // try compilation if context is already present
    if (context_) {
      for (auto pass : new_passes) {
        if (!pass->pre_compile_shaders(*context_)) {
          compilation_succeeded = false;
          break;
        }
      }
    }

    if (compilation_succeeded) {

      for (auto pass : passes_) {
        if (context_) pass->cleanup(*context_);
        delete pass;
      }

      passes_.clear();

      passes_.push_back(pre_pass);
      passes_.push_back(light_pass);
      passes_.push_back(final_pass);
      passes_.push_back(composite_pass);
      passes_.push_back(post_fx_pass);

      buffers_need_reload_ = true;

    } else {

      Logger::LOG_WARNING << "Failed to recompile shaders!" << std::endl;

      for (auto pass : new_passes) {
        if (context_) {
          pass->print_shaders("shader_compile_failed", "bla.txt");
        }
      }

      for (auto pass : new_passes) {
        if (context_) pass->cleanup(*context_);
        delete pass;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::create_buffers() {

  if (buffers_need_reload_) {

    if (passes_need_reload_ || !validate_resolution()) {
      return;
    }

    std::vector<std::shared_ptr<StereoBuffer>> stereobuffers;

    passes_[PipelineStage::geometry]->create(*context_, passes_[PipelineStage::geometry]->get_gbuffer_mapping()->get_layers());
    stereobuffers.push_back(passes_[PipelineStage::geometry]->get_gbuffer());

    passes_[PipelineStage::lighting]->create(*context_, passes_[PipelineStage::lighting]->get_gbuffer_mapping()->get_layers());
    passes_[PipelineStage::lighting]->set_inputs(stereobuffers);
    stereobuffers.push_back(passes_[PipelineStage::lighting]->get_gbuffer());

    passes_[PipelineStage::shading]->create(*context_, passes_[PipelineStage::shading]->get_gbuffer_mapping()->get_layers());
    passes_[PipelineStage::shading]->set_inputs(stereobuffers);
    stereobuffers.push_back(passes_[PipelineStage::shading]->get_gbuffer());

    passes_[PipelineStage::compositing]->set_inputs(stereobuffers);
    passes_[PipelineStage::compositing]->create(*context_, {} );
    stereobuffers.push_back(passes_[PipelineStage::compositing]->get_gbuffer());

    scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_LINEAR,
                                      scm::gl::WRAP_MIRRORED_REPEAT,
                                      scm::gl::WRAP_MIRRORED_REPEAT);

    passes_[PipelineStage::postfx]->create(*context_, { { BufferComponent::F3, state } });
    passes_[PipelineStage::postfx]->set_inputs(stereobuffers);

    if (!config.get_enable_stereo()) {
      TextureDatabase::instance()->add(config.output_texture_name(), passes_[PipelineStage::postfx]->get_gbuffer()->get_eye_buffers()[0]->get_color_buffers(TYPE_FLOAT)[0]);
      TextureDatabase::instance()->add(config.output_texture_name() + "_depth", passes_[PipelineStage::geometry]->get_gbuffer()->get_eye_buffers()[0]->get_depth_buffer());
    } else {
      TextureDatabase::instance()->add(config.output_texture_name() + "_left",  passes_[PipelineStage::postfx]->get_gbuffer()->get_eye_buffers()[0]->get_color_buffers(TYPE_FLOAT)[0]);
      TextureDatabase::instance()->add(config.output_texture_name() + "_right", passes_[PipelineStage::postfx]->get_gbuffer()->get_eye_buffers()[1]->get_color_buffers(TYPE_FLOAT)[0]);
      TextureDatabase::instance()->add(config.output_texture_name() + "_depth_left", passes_[PipelineStage::geometry]->get_gbuffer()->get_eye_buffers()[0]->get_depth_buffer());
      TextureDatabase::instance()->add(config.output_texture_name() + "_depth_right", passes_[PipelineStage::geometry]->get_gbuffer()->get_eye_buffers()[1]->get_depth_buffer());
    }

    buffers_need_reload_ = false;
  }
}

////////////////////////////////////////////////////////////////////////////////

GBufferPass::GeometryUberShaderMap const& Pipeline::get_geometry_ubershaders() const
{
  GBufferPass* gbuffpass = dynamic_cast<GBufferPass*>(passes_[geometry]);
  if (gbuffpass) {
    return gbuffpass->get_geometry_ubershaders();
  }
  else {
    throw std::runtime_error("Pipeline::get_geometry_ubershaders() : GBufferPass not created yet.");
  }
}

}
