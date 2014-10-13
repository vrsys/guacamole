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
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/WindowBase.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/scenegraph/SceneGraph.hpp>
#include <gua/renderer/Serializer.hpp>

// external headers
#include <iostream>

namespace gua {

namespace {

////////////////////////////////////////////////////////////////////////////////

Frustum camera_frustum(Camera::ProjectionMode const& mode,
    math::mat4 const& transf, math::mat4 const& screen,
    float near, float far) {
  if (mode == Camera::ProjectionMode::PERSPECTIVE) {
    return Frustum::perspective(transf, screen, near, far);
  } else {
    return Frustum::orthographic(transf, screen, near, far);
  }
}

////////////////////////////////////////////////////////////////////////////////

void serialize(SceneGraph const& scene_graph,
               std::string const& eye_name,
               std::string const& screen_name,
               Pipeline::Configuration const& config,
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

  Serializer serializer;
  serializer.check(out, scene_graph,
                   config.camera().render_mask,
                   config.enable_bbox_display(),
                   config.enable_ray_display(),
                   config.enable_frustum_culling());
}

////////////////////////////////////////////////////////////////////////////////


}



////////////////////////////////////////////////////////////////////////////////

Pipeline::Pipeline() :
  gbuffer_(nullptr),
  camera_block_(nullptr),
  last_resolution_(0, 0),
  dirty_(true),
  fullscreen_quad_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

Pipeline::~Pipeline() {
  for (auto pass: passes_) {
    delete pass;
  }

  if (camera_block_) {
    delete camera_block_;
  }
}

////////////////////////////////////////////////////////////////////////////////

std::list<PipelinePass*> const& Pipeline::get_passes() const {
  return passes_;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::process(std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
                       float application_fps, float rendering_fps) {

  std::cout << "App: " << application_fps << " Render: " << rendering_fps << std::endl;

  // return if pipeline is disabled
  if (!config.get_enabled()) {
    return;
  }

  // update window if one is assigned
  if (window_) {
    if (!window_->get_is_open()) {
      window_->open();
      window_->create_shader();
    }
    window_->set_active(true);
  }

  if (last_resolution_ != window_->config.size()) {
    last_resolution_ = window_->config.size();
    dirty_ = true;
  }

  // recreate gbuffer if resolution changed
  if (dirty_) {
    if (gbuffer_) {
      gbuffer_->remove_buffers(get_context());
      delete gbuffer_;
    }

    gbuffer_ = new GBuffer(get_context(), config.resolution().x, config.resolution().y);
    dirty_ = false;
  }

  // get scenegraph which shall be rendered
  SceneGraph const* current_graph = nullptr;
  for (auto& graph: scene_graphs) {
    if (graph->get_name() == config.camera().scene_graph) {
      current_graph = graph.get();
      break;
    }
  }
  // update camera uniform block
  if (!camera_block_) {
    camera_block_ = new CameraUniformBlock(get_context().render_device);
  }

  auto process_passes = [&, this](bool is_left) {

    // serialize this scenegraph
    serialize(*current_graph,
              is_left ? config.camera().eye_l : config.camera().eye_r,
              is_left ? config.camera().screen_l : config.camera().screen_r,
              config, current_scene_);

    camera_block_->update(get_context().render_context, current_scene_.frustum);
    bind_camera_uniform_block(0);

    // clear gbuffer
    gbuffer_->clear_all(get_context());

    // process all passes
    for (auto pass: passes_) {

      if (pass->needs_color_buffer_as_input()) {
        gbuffer_->toggle_ping_pong();
      }

      pass->process(this);
    }

    // display the last written colorbuffer of the gbuffer
    if (window_) {
      gbuffer_->toggle_ping_pong();
      window_->display(gbuffer_->get_current_color_buffer(), is_left);
    }
  };

  process_passes(true);

  if (config.get_enable_stereo()) {
    process_passes(false);
  }

  // swap buffers
  if (window_) {
    window_->finish_frame();
  }
  // ++(get_context().framecount);
}

////////////////////////////////////////////////////////////////////////////////

GBuffer& Pipeline::get_gbuffer() const {
  return *gbuffer_;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::set_output_window(WindowBase* window) {
  window_ = window;
}

////////////////////////////////////////////////////////////////////////////////

RenderContext const& Pipeline::get_context() const {
  return *window_->get_context();
}

////////////////////////////////////////////////////////////////////////////////

SerializedScene const& Pipeline::get_scene() const {
  return current_scene_;
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
  get_context().render_context->bind_uniform_buffer(camera_block_->block().block_buffer(), location);
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::draw_fullscreen_quad() {
  if (!fullscreen_quad_) {
    fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(
              get_context().render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));
  }

  fullscreen_quad_->draw(get_context().render_context);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<RessourceRenderer> Pipeline::get_renderer(GeometryResource const& type) {
  std::type_index id = typeid(type);
  auto renderer = renderers_.find(id);

  if (renderer != renderers_.end()) {
    return renderer->second;
  }

  auto new_renderer = type.create_renderer();
  renderers_[id] = new_renderer;

  return new_renderer;
}

////////////////////////////////////////////////////////////////////////////////

}
