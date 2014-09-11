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
  dirty_(false) {}

////////////////////////////////////////////////////////////////////////////////

Pipeline::~Pipeline() {
  for (auto pass: passes_) {
    delete pass;
  }
}

////////////////////////////////////////////////////////////////////////////////

std::list<PipelinePass*> const& Pipeline::get_passes() const {
  return passes_;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::process(std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
                       float application_fps, float rendering_fps) {

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

  // serialize this scenegraph
  serialize(*current_graph,
            config.camera().eye_l, config.camera().screen_l,
            config, current_scene_);

  // clear gbuffer
  gbuffer_->clear(get_context());

  // process all passes
  for (auto pass: passes_) {
    if (pass->use_last_color_buffer()) {
      gbuffer_->toggle_ping_pong();
    }

    pass->process(this);
  }

  // display the last written colorbuffer of the gbuffer
  if (window_) {
    // gbuffer_->toggle_ping_pong();
    window_->display(gbuffer_->get_color_buffer());
  }

  // swap buffers
  window_->finish_frame();
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
