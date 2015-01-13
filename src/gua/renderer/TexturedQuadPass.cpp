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
#include <gua/renderer/TexturedQuadPass.hpp>

#include <gua/node/TexturedQuadNode.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/Resources.hpp>

namespace gua {

TexturedQuadPassDescription::TexturedQuadPassDescription()
  : PipelinePassDescription() {

  vertex_shader_ = "resources/shaders/common/quad.vert";
  fragment_shader_ = "resources/shaders/textured_quad.frag";

  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = false;
  doClear_ = false;
  rendermode_ = RenderMode::Callback;

  rasterizer_state_ = boost::make_optional(scm::gl::rasterizer_state_desc(
        scm::gl::FILL_SOLID, scm::gl::CULL_NONE));

  process_ = [](
      PipelinePass & pass, PipelinePassDescription const&, Pipeline & pipe) {

    for (auto const& node : pipe.get_scene().nodes[std::type_index(typeid(node::TexturedQuadNode))]) {
      auto quad_node(reinterpret_cast<node::TexturedQuadNode*>(node));

      UniformValue model_mat(quad_node->get_scaled_world_transform());
      UniformValue normal_mat(scm::math::transpose(scm::math::inverse(quad_node->get_scaled_world_transform())));
      UniformValue tex(quad_node->data.get_texture());
      UniformValue flip(math::vec2i(quad_node->data.get_flip_x() ? -1 : 1, quad_node->data.get_flip_y() ? -1 : 1));

      auto const& ctx(pipe.get_context());

      pass.shader_->apply_uniform(ctx, "gua_model_matrix", model_mat);
      pass.shader_->apply_uniform(ctx, "gua_normal_matrix", normal_mat);
      pass.shader_->apply_uniform(ctx, "gua_in_texture", tex);
      pass.shader_->apply_uniform(ctx, "flip", flip);

      pipe.draw_quad();
    }
  };
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* TexturedQuadPassDescription::make_copy() const {
  return new TexturedQuadPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass TexturedQuadPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
  PipelinePass pass{*this, ctx, substitution_map};
  return pass;
}

}
