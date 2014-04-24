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
#include <gua/renderer/Pass.hpp>

// guacamole headers
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/utils.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Pass::Pass(Pipeline* pipeline)
  : pipeline_(pipeline)
  , gbuffer_(nullptr)
  , inputs_()
  , initialized_(false)
  {}

////////////////////////////////////////////////////////////////////////////////

void Pass::create(
    RenderContext const& ctx,
    std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> > const&
        layers) {

  if (gbuffer_) {
    gbuffer_->remove_buffers(ctx);
  }

  gbuffer_ = std::make_shared<StereoBuffer>(ctx, pipeline_->config, layers);
}

////////////////////////////////////////////////////////////////////////////////

void Pass::set_inputs(std::vector<std::shared_ptr<StereoBuffer>> inputs) {

  inputs_ = inputs;
}

void Pass::bind_inputs(ShaderProgram const& shader,
                       CameraMode eye,
                       RenderContext const& ctx) const {

  shader.set_uniform(
      ctx,
      1.0f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(),
      "gua_texel_width");
  shader.set_uniform(
      ctx,
      1.0f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(),
      "gua_texel_height");
  shader.set_uniform(ctx,
                     pipeline_->config.get_tesselation_max_error(),
                     "gua_tesselation_max_error");

  for (unsigned stage(0); stage < inputs_.size(); ++stage) {
    auto bind_input = [&](std::string const & name, BufferComponentType type) {
      auto buffers(inputs_[stage]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]
                       ->get_color_buffers(type));
      for (unsigned buffer_number(0); buffer_number < buffers.size();
           ++buffer_number) {
        shader.set_uniform(ctx,
                           buffers[buffer_number],
                           name + string_utils::to_string(stage + 1),
                           buffer_number);
      }
    }
    ;

    bind_input("gua_int_gbuffer_in_", TYPE_INTEGER);
    bind_input("gua_uint_gbuffer_in_", TYPE_UNSIGNED);
    bind_input("gua_half_gbuffer_in_", TYPE_HALF);
    bind_input("gua_float_gbuffer_in_", TYPE_FLOAT);
  }

  if (inputs_.size() > 0) {
    auto depth_buffer(inputs_[0]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]
                          ->get_depth_buffer());
    shader.set_uniform(ctx, depth_buffer, "gua_depth_gbuffer_in");
  }
}

////////////////////////////////////////////////////////////////////////////////

void Pass::set_camera_matrices(ShaderProgram const& shader,
                               Camera const& camera,
                               SerializedScene const& scene,
                               CameraMode eye,
                               RenderContext const& ctx) const {

  auto camera_position(scene.frustum.get_camera_position());
  auto projection(scene.frustum.get_projection());
  auto view_matrix(scene.frustum.get_view());
  auto inv_projection(scm::math::inverse(projection));

  shader.set_uniform(ctx, camera_position, "gua_camera_position");
  shader.set_uniform(ctx, projection, "gua_projection_matrix");
  shader.set_uniform(ctx, inv_projection, "gua_inverse_projection_matrix");
  shader.set_uniform(ctx, view_matrix, "gua_view_matrix");
  shader.set_uniform(ctx,
                     scm::math::inverse(projection * view_matrix),
                     "gua_inverse_projection_view_matrix");
}

////////////////////////////////////////////////////////////////////////////////

}
