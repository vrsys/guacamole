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
#include <gua/scenegraph/SceneGraph.hpp>

// external headers
#include <iostream>

namespace {

// gua::Frustum camera_frustum(gua::Camera::ProjectionMode const& mode,
//     gua::math::mat4 const& transf, gua::math::mat4 const& screen,
//     float near, float far) {
//   if (mode == gua::Camera::ProjectionMode::PERSPECTIVE) {
//     return gua::Frustum::perspective(transf, screen, near, far);
//   } else {
//     return gua::Frustum::orthographic(transf, screen, near, far);
//   }
// }

}


namespace gua {

////////////////////////////////////////////////////////////////////////////////

Pipeline::Pipeline() :
  gbuffer_(nullptr),
  dirty_(false) {
  
}

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

  // process all passes
  for (auto pass: passes_) {
    pass->process(this);
  }

}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::bind_gbuffer() const {
  gbuffer_->bind(get_context());
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

}
