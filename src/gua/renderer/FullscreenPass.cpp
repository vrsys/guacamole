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
#include <gua/renderer/FullscreenPass.hpp>

// guacamole headers
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/databases.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

FullscreenPass::FullscreenPass(Pipeline* pipeline)
    : Pass(pipeline), fullscreen_quad_(), depth_stencil_state_() {}

////////////////////////////////////////////////////////////////////////////////

FullscreenPass::~FullscreenPass() {}

////////////////////////////////////////////////////////////////////////////////

void FullscreenPass::render_scene(Camera const& camera,
                                  SceneGraph const*,
                                  RenderContext const& ctx) {

  if (!depth_stencil_state_)
    depth_stencil_state_ = ctx.render_device
        ->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);

  if (!fullscreen_quad_)
    fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(
        ctx.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));
  set_uniforms(pipeline_->get_current_scene(CameraMode::CENTER), ctx);

  for (int i(0); i < gbuffer_->get_eye_buffers().size(); ++i) {
    CameraMode eye(CameraMode::CENTER);

    if (gbuffer_->get_eye_buffers().size() > 1 && i == 0)
      eye = CameraMode::LEFT;
    if (gbuffer_->get_eye_buffers().size() > 1 && i == 1)
      eye = CameraMode::RIGHT;

    pre_rendering(camera, pipeline_->get_current_scene(eye), eye, ctx);

    FrameBufferObject* fbo(gbuffer_->get_eye_buffers()[i]);
    fbo->bind(ctx);

    ctx.render_context->set_viewport(scm::gl::viewport(
        math::vec2(0, 0), math::vec2(float(fbo->width()), float(fbo->height()))));
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_);

    rendering(camera, pipeline_->get_current_scene(eye), eye, ctx);

    ctx.render_context->reset_state_objects();

    fbo->unbind(ctx);

    post_rendering(camera, pipeline_->get_current_scene(eye), eye, ctx);
  }

}

////////////////////////////////////////////////////////////////////////////////

}
