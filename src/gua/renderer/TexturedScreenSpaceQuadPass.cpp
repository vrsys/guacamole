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
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>

#include <gua/node/TexturedScreenSpaceQuadNode.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/Resources.hpp>

namespace gua {

TexturedScreenSpaceQuadPassDescription::TexturedScreenSpaceQuadPassDescription()
  : PipelinePassDescription() {

  vertex_shader_ = "shaders/textured_screen_space_quad.vert";
  fragment_shader_ = "shaders/textured_screen_space_quad.frag";

  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = true;
  doClear_ = false;
  rendermode_ = RenderMode::Callback;

  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));

  blend_state_ = boost::make_optional(
      scm::gl::blend_state_desc(scm::gl::blend_ops(true,
                                                   scm::gl::FUNC_SRC_ALPHA,
                                                   scm::gl::FUNC_ONE_MINUS_SRC_ALPHA,
                                                   scm::gl::FUNC_SRC_ALPHA,
                                                   scm::gl::FUNC_ONE_MINUS_SRC_ALPHA)));

  process_ = [](
      PipelinePass & pass, PipelinePassDescription const&, Pipeline & pipe) {

    for (auto const& node : pipe.get_scene().nodes[std::type_index(typeid(node::TexturedScreenSpaceQuadNode))]) {
      auto quad_node(reinterpret_cast<node::TexturedScreenSpaceQuadNode*>(node));

      UniformValue tex(quad_node->data.get_texture());
      UniformValue flip(math::vec2i(quad_node->data.get_flip_x() ? -1 : 1, quad_node->data.get_flip_y() ? -1 : 1));
      UniformValue opacity(quad_node->data.get_opacity());

      auto width(pipe.get_gbuffer().get_width());
      auto height(pipe.get_gbuffer().get_height());

      UniformValue size( math::vec2(
        1.0 * quad_node->data.get_size().x / width,
        1.0 * quad_node->data.get_size().y / height
      ));

      UniformValue offset( math::vec2(
        (2.0 * quad_node->data.get_offset().x + quad_node->data.get_anchor().x * (width - quad_node->data.get_size().x))/width,
        (2.0 * quad_node->data.get_offset().y + quad_node->data.get_anchor().y * (height - quad_node->data.get_size().y))/height
      ));

      auto const& ctx(pipe.get_context());

      pass.shader_->apply_uniform(ctx, "gua_in_texture", tex);
      pass.shader_->apply_uniform(ctx, "flip", flip);
      pass.shader_->apply_uniform(ctx, "size", size);
      pass.shader_->apply_uniform(ctx, "offset", offset);
      pass.shader_->apply_uniform(ctx, "opacity", opacity);

      pipe.draw_quad();
    }
  };
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* TexturedScreenSpaceQuadPassDescription::make_copy() const {
  return new TexturedScreenSpaceQuadPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

}
