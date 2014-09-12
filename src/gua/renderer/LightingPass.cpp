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
#include <gua/renderer/LightingPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

LightingPass::LightingPass() :
  shader_(nullptr),
  light_sphere_(nullptr),
  rasterizer_state_front_(nullptr),
  depth_stencil_state_(nullptr),
  blend_state_(nullptr) {}

void LightingPass::process(Pipeline* pipe) {

  RenderContext const& ctx(pipe->get_context());

  // init resources
  if (!shader_) {
    shader_ = std::make_shared<ShaderProgram>();
    shader_->create_from_sources(
      Resources::lookup_shader(Resources::shaders_lighting_vert), 
      Resources::lookup_shader(Resources::shaders_lighting_frag)
    );
  }

  if (!light_sphere_) {
    light_sphere_ = GeometryDatabase::instance()->lookup("gua_light_sphere_proxy");
  }

  if (!depth_stencil_state_) {
    depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(false, false);
  }

  if (!rasterizer_state_front_) {
    rasterizer_state_front_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT);
  }

  // if (!rasterizer_state_back_) {
  //   rasterizer_state_back_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);
  // }

  if (!blend_state_) {
    blend_state_ = ctx.render_device->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE);
  }

  // bind gbuffer
  pipe->get_gbuffer().bind(ctx, true);
  pipe->get_gbuffer().set_viewport(ctx);

  // set state
  ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
  ctx.render_context->set_rasterizer_state(rasterizer_state_front_);
  ctx.render_context->set_blend_state(blend_state_);

  shader_->use(ctx);
  shader_->set_subroutine(ctx, scm::gl::STAGE_VERTEX_SHADER,   "compute_light", "gua_calculate_point_light");
  shader_->set_subroutine(ctx, scm::gl::STAGE_FRAGMENT_SHADER, "compute_light", "gua_calculate_point_light");
  
  pipe->bind_gbuffer_input(shader_);

  // draw all lights
  for (auto const& light : pipe->get_scene().point_lights_) {
    shader_->set_uniform(ctx, light->get_cached_world_transform(),        "gua_model_matrix");
    shader_->set_uniform(ctx, light->data.get_enable_diffuse_shading(),   "gua_light_diffuse_enable");
    shader_->set_uniform(ctx, light->data.get_enable_specular_shading(),  "gua_light_specular_enable");
    shader_->set_uniform(ctx, light->data.get_color().vec3(),             "gua_light_color");
    shader_->set_uniform(ctx, light->data.get_falloff(),                  "gua_light_falloff");
    shader_->set_uniform(ctx, false,                                      "gua_light_casts_shadow");
    light_sphere_->draw(ctx);
  }

  pipe->get_gbuffer().unbind(ctx);

  ctx.render_context->reset_state_objects();
} 

}
