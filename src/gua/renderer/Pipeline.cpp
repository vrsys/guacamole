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

#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/renderer/LightTable.hpp>

// external headers
#include <iostream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Pipeline::Pipeline() :
  gbuffer_(nullptr),
  abuffer_(),
  camera_block_(nullptr),
  light_table_(nullptr),
  last_resolution_(0, 0),
  quad_(nullptr),
  context_(nullptr) {


}

////////////////////////////////////////////////////////////////////////////////

Pipeline::~Pipeline() {
  if (light_table_) {
    delete light_table_;
  }
  if (camera_block_) {
    delete camera_block_;
  }
  if (gbuffer_) {
    delete gbuffer_;
  }
}

////////////////////////////////////////////////////////////////////////////////

std::vector<PipelinePass> const& Pipeline::get_passes() const {
  return passes_;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::process(RenderContext* ctx, CameraMode mode, node::SerializedCameraNode const& camera,
                       std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs) {

  // return if pipeline is disabled
  if (!camera.config.get_enabled()) {
    return;
  }

  // store the current camera data
  current_camera_ = camera;

  bool context_changed(false);
  bool reload_gbuffer(false);
  bool reload_abuffer(false);

  // reload gbuffer if now rendering to another window (with a new context)
  if (context_ != ctx) {
    context_ = ctx;
    context_changed = true;
  }

  // execute all prerender cameras
  for (auto const& cam: camera.pre_render_cameras) {
    cam.rendering_pipeline->process(ctx, mode, cam, scene_graphs);
  }

  // recreate gbuffer if resolution changed
  if (last_resolution_ != camera.config.get_resolution()) {
    last_resolution_ = camera.config.get_resolution();
    reload_gbuffer = true;
  }

  if (context_changed || reload_gbuffer) {
    if (gbuffer_) {
      gbuffer_->remove_buffers(get_context());
      delete gbuffer_;
    }

    gbuffer_ = new GBuffer(get_context(), camera.config.resolution());
  }


  // recreate pipeline passes if pipeline description changed
  bool reload_passes(context_changed || reload_gbuffer);

  if (*camera.pipeline_description != last_description_) {
    reload_passes = true;
    reload_abuffer = true;
    last_description_ = *camera.pipeline_description;
  } else {

    // if pipeline configuration is unchanged, update only uniforms of passes
    for (int i(0); i < last_description_.get_all_passes().size(); ++i) {
      last_description_.get_all_passes()[i]->uniforms = camera.pipeline_description->get_all_passes()[i]->uniforms;
    }
  }

  if (context_changed || reload_abuffer) {
    abuffer_.allocate(get_context(),
                      last_description_.get_enable_abuffer()
                        ? last_description_.get_abuffer_size() : 0);
  }

  if (reload_passes) {
    for (auto & pass: passes_) {
      pass.on_delete(this);
    }

    passes_.clear();
    global_substitution_map_.clear();

    global_substitution_map_["enable_abuffer"] = last_description_.get_enable_abuffer() ? "1" : "0";
    global_substitution_map_["abuf_insertion_threshold"] = "0.9995";
    global_substitution_map_["abuf_blending_termination_threshold"] = "0.9995";
    global_substitution_map_["max_lights_num"] = "128";

    for (auto pass: last_description_.get_all_passes()) {
      passes_.push_back(pass->make_pass(*ctx, global_substitution_map_));
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

  if (context_changed) {
    // update camera uniform block
    if (camera_block_) {
      delete camera_block_;
    }
    camera_block_ = new CameraUniformBlock(get_context().render_device);

    // update light table
    if (light_table_) {
      light_table_->remove_buffers(get_context());
      delete light_table_;
    }
    light_table_ = new LightTable();
  }

  context_->mode = mode;

  // serialize this scenegraph
  current_scene_ = current_graph_->serialize(camera, mode);

  camera_block_->update(get_context().render_context, current_scene_.frustum);
  bind_camera_uniform_block(0);

  // clear gbuffer and abuffer
  gbuffer_->clear_all(get_context());
  abuffer_.clear(get_context(), camera.config.resolution());

  // process all passes
  for (int i(0); i < passes_.size(); ++i) {

    if (passes_[i].needs_color_buffer_as_input()) {
      gbuffer_->toggle_ping_pong();
    }

    passes_[i].process(*last_description_.get_all_passes()[i], *this);
  }

  gbuffer_->toggle_ping_pong();

  // add texture to texture database
  auto const& tex(gbuffer_->get_current_color_buffer());
  auto tex_name(camera.config.get_output_texture_name());

  if (tex_name != "") {
    TextureDatabase::instance()->add(tex_name, tex);
  }

  if (camera.config.get_output_window_name() != "") {
    auto window = WindowDatabase::instance()->lookup(camera.config.get_output_window_name());
    if (window) {
      window->display(tex, mode != CameraMode::RIGHT);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

GBuffer& Pipeline::get_gbuffer() const {
  return *gbuffer_;
}

////////////////////////////////////////////////////////////////////////////////

ABuffer& Pipeline::get_abuffer() {
  return abuffer_;
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

LightTable& Pipeline::get_light_table() {
  return *light_table_;
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::bind_gbuffer_input(std::shared_ptr<ShaderProgram> const& shader) const {

  auto& ctx(get_context());

  shader->set_uniform(ctx, 1.0f / gbuffer_->get_width(),  "gua_texel_width");
  shader->set_uniform(ctx, 1.0f / gbuffer_->get_height(), "gua_texel_height");

  shader->set_uniform(ctx, math::vec2i(gbuffer_->get_width(), gbuffer_->get_height()),  "gua_resolution");

  shader->set_uniform(ctx, gbuffer_->get_current_color_buffer()->get_handle(ctx),  "gua_gbuffer_color");
  shader->set_uniform(ctx, gbuffer_->get_current_pbr_buffer()->get_handle(ctx),    "gua_gbuffer_pbr");
  shader->set_uniform(ctx, gbuffer_->get_current_normal_buffer()->get_handle(ctx), "gua_gbuffer_normal");
  shader->set_uniform(ctx, gbuffer_->get_current_flags_buffer()->get_handle(ctx),  "gua_gbuffer_flags");
  shader->set_uniform(ctx, gbuffer_->get_current_depth_buffer()->get_handle(ctx),  "gua_gbuffer_depth");
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::bind_light_table(std::shared_ptr<ShaderProgram> const& shader) const {

  auto& ctx(get_context());

  shader->set_uniform(ctx, int(light_table_->get_lights_num()), "gua_lights_num");
  if (   light_table_->get_light_bitset()
      && light_table_->get_lights_num() > 0) {
    shader->set_uniform(ctx, light_table_->get_light_bitset()->get_handle(ctx), "gua_light_bitset");
    ctx.render_context->bind_uniform_buffer(light_table_->light_uniform_block().block_buffer(), 1);
  }
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
