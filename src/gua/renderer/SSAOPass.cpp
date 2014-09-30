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
#include <gua/renderer/SSAOPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

SSAOPass::SSAOPass() :
  shader_(std::make_shared<ShaderProgram>()),
  depth_stencil_state_(nullptr),
  blend_state_(nullptr),
  noise_texture_(),
  radius_(1.f),
  intensity_(1.f),
  falloff_(0.1f) {

  shader_ = std::make_shared<ShaderProgram>();
  shader_->create_from_sources(
    Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_vert),
    Resources::lookup_shader(Resources::shaders_ssao_frag)
  );
}

////////////////////////////////////////////////////////////////////////////////

float SSAOPass::radius() const{
  return radius_;
}

////////////////////////////////////////////////////////////////////////////////

SSAOPass& SSAOPass::radius(float radius) {
  radius_ = radius;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

float SSAOPass::intensity() const{
  return intensity_;
}

////////////////////////////////////////////////////////////////////////////////

SSAOPass& SSAOPass::intensity(float intensity) {
  intensity_ = intensity;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

float SSAOPass::falloff() const{
  return falloff_;
}

////////////////////////////////////////////////////////////////////////////////

SSAOPass& SSAOPass::falloff(float falloff) {
  falloff_ = falloff;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

void SSAOPass::process(Pipeline* pipe) {
  RenderContext const& ctx(pipe->get_context());

  if (!depth_stencil_state_) {
    depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(false, false);
  }

  if (!blend_state_) {
    blend_state_ = ctx.render_device->create_blend_state(
      true,
      scm::gl::FUNC_SRC_ALPHA, scm::gl::FUNC_ONE_MINUS_SRC_ALPHA,
      scm::gl::FUNC_SRC_ALPHA, scm::gl::FUNC_ONE_MINUS_SRC_ALPHA
    );
  }

  // bind gbuffer
  pipe->get_gbuffer().bind(ctx, this);
  pipe->get_gbuffer().set_viewport(ctx);

  ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
  ctx.render_context->set_blend_state(blend_state_);

  shader_->use(ctx);

  shader_->set_uniform(ctx, noise_texture_.get_handle(ctx), "gua_noise_tex");
  shader_->set_uniform(ctx, radius_,    "gua_ssao_radius");
  shader_->set_uniform(ctx, intensity_, "gua_ssao_intensity");
  shader_->set_uniform(ctx, falloff_,   "gua_ssao_falloff");

  pipe->bind_gbuffer_input(shader_);
  pipe->draw_fullscreen_quad();
  pipe->get_gbuffer().unbind(ctx);

  ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

}
