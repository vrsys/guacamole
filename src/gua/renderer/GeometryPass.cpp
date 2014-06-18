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
#include <gua/renderer/GeometryPass.hpp>

// guacamole headers
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/View.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

GeometryPass::GeometryPass(Pipeline* pipeline) : Pass(pipeline) {}

////////////////////////////////////////////////////////////////////////////////

void GeometryPass::render_scene(Camera const& camera,
                                SceneGraph const& current_graph,
                                RenderContext const& ctx,
                                std::size_t viewid) {

  gbuffer_->clear(ctx);

  bool is_stereo = gbuffer_->get_eye_buffers().size() > 1;
  View view(viewid, is_stereo);

  if (is_stereo) // stereo -> todo: combine frusta?
  {
    view.left  = pipeline_->get_current_scene(CameraMode::LEFT).frustum;
    view.right = pipeline_->get_current_scene(CameraMode::RIGHT).frustum;
  } else {       // mono -> just use left frustum
    view.left = pipeline_->get_current_scene(CameraMode::LEFT).frustum;
  }

  for (int i(0); i < gbuffer_->get_eye_buffers().size(); ++i) {

    FrameBufferObject* fbo(gbuffer_->get_eye_buffers()[i]);

    CameraMode eye;
    if (!is_stereo) {
      eye = CameraMode::CENTER;
    } else {
      eye = (i == 0) ? CameraMode::LEFT : CameraMode::RIGHT;
    }

    fbo->bind(ctx);

    ctx.render_context->set_viewport(scm::gl::viewport(
        math::vec2(0, 0), ::scm::math::vec2f(fbo->width(), fbo->height())));

    rendering(pipeline_->get_current_scene(eye), current_graph, ctx, eye, camera, fbo, view);

    fbo->unbind(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

}
