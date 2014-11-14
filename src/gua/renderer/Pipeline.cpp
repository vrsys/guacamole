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
#include <gua/node/CameraNode.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/WindowBase.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/node/CameraNode.hpp>
#include <gua/scenegraph/SceneGraph.hpp>
#include <gua/renderer/Serializer.hpp>

// external headers
#include <iostream>

namespace gua {

namespace {

////////////////////////////////////////////////////////////////////////////////

Frustum camera_frustum(node::CameraNode::ProjectionMode const& mode,
    math::mat4 const& transf, math::mat4 const& screen,
    float near, float far) {
  if (mode == node::CameraNode::ProjectionMode::PERSPECTIVE) {
    return Frustum::perspective(transf, screen, near, far);
  } else {
    return Frustum::orthographic(transf, screen, near, far);
  }
}

////////////////////////////////////////////////////////////////////////////////

void serialize(SceneGraph const& scene_graph, bool is_left,
               node::SerializedCameraNode const& camera, SerializedScene& out) {

  std::string screen_name(is_left ? camera.config.left_screen_path() : camera.config.right_screen_path());
  auto screen_it((scene_graph)[screen_name]);
  auto screen(std::dynamic_pointer_cast<node::ScreenNode>(screen_it));
  if (!screen) {
    Logger::LOG_WARNING << "Cannot render scene: No valid screen specified" << std::endl;
    return;
  }

  auto eye_transform(camera.transform);

  if (camera.config.get_enable_stereo()) {
    if (is_left) {
      eye_transform *= scm::math::make_translation(camera.config.eye_offset() - 0.5f * camera.config.eye_dist(), 0.f, 0.f);
    } else {
      eye_transform *= scm::math::make_translation(camera.config.eye_offset() + 0.5f * camera.config.eye_dist(), 0.f, 0.f);
    }
  }
  camera.config.eye_dist(), camera.config.eye_offset();

  out.frustum = camera_frustum(camera.config.mode(), eye_transform,
                               screen->get_scaled_world_transform(),
                               camera.config.near_clip(),
                               camera.config.far_clip());

  out.center_of_interest = math::get_translation(camera.transform);

  Serializer serializer;
  serializer.check(out, scene_graph,
                   camera.config.mask(),
                   camera.config.enable_frustum_culling());
}

////////////////////////////////////////////////////////////////////////////////

}

////////////////////////////////////////////////////////////////////////////////

Pipeline::Pipeline() :
  fps_count_(0),
  fps_sum_(0.f, 0.f),
  gbuffer_(nullptr),
  camera_block_(nullptr),
  last_resolution_(0, 0),
  quad_(nullptr),
  context_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

Pipeline::~Pipeline() {
  if (camera_block_) {
    delete camera_block_;
  }
}

////////////////////////////////////////////////////////////////////////////////

std::vector<PipelinePass> const& Pipeline::get_passes() const {
  return passes_;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::process(RenderContext* ctx, node::SerializedCameraNode const& camera,
                       std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
                       float application_fps, float rendering_fps) {

  if (fps_count_ > 100) {
    std::cout << "App: " << fps_sum_[0]/fps_count_ << " Render: " << fps_sum_[1]/fps_count_ << std::endl;
    fps_count_ = 0;
    fps_sum_ = math::vec2(0.f, 0.f);
  } else {
    ++fps_count_;
    fps_sum_[0] += application_fps;
    fps_sum_[1] += rendering_fps;
  }

  // return if pipeline is disabled
  if (!camera.config.get_enabled()) {
    return;
  }

  // store the current camera data
  current_camera_ = camera;

  bool reload_gbuffer(false);

  // reload gbuffer if now rendering to another window (with a new context)
  if (context_ != ctx) {
    context_ = ctx;
    reload_gbuffer = true;
  }

  // execute all prerender cameras
  for (auto const& cam: camera.pre_render_cameras) {
    cam.rendering_pipeline->process(ctx, cam, scene_graphs, application_fps, rendering_fps);
  }

  // recreate gbuffer if resolution changed
  if (last_resolution_ != camera.config.get_resolution()) {
    last_resolution_ = camera.config.get_resolution();
    reload_gbuffer = true;
  }

  if (reload_gbuffer) {
    if (gbuffer_) {
      gbuffer_->remove_buffers(get_context());
      delete gbuffer_;
    }

    gbuffer_ = new GBuffer(get_context(), camera.config.resolution());
  }

  // recreate pipeline passes if pipeline description changed
  bool reload_passes(reload_gbuffer);

  if (camera.config.get_pipeline_description() != last_description_) {
    last_description_ = camera.config.get_pipeline_description();
    reload_passes = true;
  }

  if (reload_passes) {
    for (auto & pass: passes_) {
      pass.on_delete(this);
    }

    passes_.clear();

    for (auto pass: camera.config.get_pipeline_description().get_passes()) {
      passes_.push_back(PipelinePass{*pass,*ctx});
    }
  }

  // get scenegraph which shall be rendered
  current_graph_ = nullptr;
  for (auto& graph: scene_graphs) {
    if (graph->get_name() == camera.config.get_scene_graph_name()) {
      current_graph_ = graph.get();
      break;
    }
  }

  // update camera uniform block
  if (!camera_block_) {
    camera_block_ = new CameraUniformBlock(get_context().render_device);
  }

  auto process_passes = [&](CameraMode mode) {

    context_->mode = mode;

    // serialize this scenegraph
    serialize(*current_graph_, mode != CameraMode::RIGHT, camera, current_scene_);

    camera_block_->update(get_context().render_context, current_scene_.frustum);
    bind_camera_uniform_block(0);

    // clear gbuffer
    gbuffer_->clear_all(get_context());

    // process all passes
    for (int i(0); i < passes_.size(); ++i) {

      if (passes_[i].needs_color_buffer_as_input()) {
        gbuffer_->toggle_ping_pong();
      }

      passes_[i].process(*camera.config.get_pipeline_description().get_passes()[i], *this);
    }

    gbuffer_->toggle_ping_pong();

    // add texture to texture database
    auto const& tex(gbuffer_->get_current_color_buffer());
    auto tex_name(camera.config.get_output_texture_name());

    if (tex_name != "") {
      if (mode == CameraMode::LEFT) {
        tex_name += "_left";
      } else if (mode == CameraMode::RIGHT) {
        tex_name += "_right";
      }

      TextureDatabase::instance()->add(tex_name, tex);
    }

    if (camera.config.get_output_window_name() != "") {
      auto window = WindowDatabase::instance()->lookup(camera.config.get_output_window_name());
      if (window) {
        window->display(tex, mode != CameraMode::RIGHT);
      }
    }
  };

  if (camera.config.get_enable_stereo()) {
    process_passes(CameraMode::LEFT);
    process_passes(CameraMode::RIGHT);
  } else {
    process_passes(camera.config.get_mono_mode());
  }
}

////////////////////////////////////////////////////////////////////////////////

GBuffer& Pipeline::get_gbuffer() const {
  return *gbuffer_;
}

////////////////////////////////////////////////////////////////////////////////

node::SerializedCameraNode const& Pipeline::get_camera() const {
  return current_camera_;
}

////////////////////////////////////////////////////////////////////////////////

RenderContext const& Pipeline::get_context() const {
  return *context_;
}

////////////////////////////////////////////////////////////////////////////////

SerializedScene& Pipeline::get_scene() {
  return current_scene_;
}

////////////////////////////////////////////////////////////////////////////////

SceneGraph const& Pipeline::get_graph() const {
  return *current_graph_;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::bind_gbuffer_input(std::shared_ptr<ShaderProgram> const& shader) const {

  auto& ctx(get_context());

  shader->set_uniform(ctx, 1.0f / gbuffer_->get_width(),  "gua_texel_width");
  shader->set_uniform(ctx, 1.0f / gbuffer_->get_height(), "gua_texel_height");

  shader->set_uniform(ctx, gbuffer_->get_current_color_buffer()->get_handle(ctx),  "gua_gbuffer_color");
  shader->set_uniform(ctx, gbuffer_->get_current_pbr_buffer()->get_handle(ctx),    "gua_gbuffer_pbr");
  shader->set_uniform(ctx, gbuffer_->get_current_normal_buffer()->get_handle(ctx), "gua_gbuffer_normal");
  shader->set_uniform(ctx, gbuffer_->get_current_depth_buffer()->get_handle(ctx),  "gua_gbuffer_depth");
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::bind_camera_uniform_block(unsigned location) const {
  get_context().render_context->bind_uniform_buffer(
    camera_block_->block().block_buffer(), location
  );
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::draw_quad() {
  if (!quad_) {
    quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(
      get_context().render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f))
    );
  }

  quad_->draw(get_context().render_context);
}

////////////////////////////////////////////////////////////////////////////////

}
