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
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/databases/WindowDatabase.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

void render_node(PipelinePass& pass, node::TexturedScreenSpaceQuadNode* quad_node, Pipeline& pipe, bool render_multiview)
{
    UniformValue tex(quad_node->data.get_texture());
    UniformValue flip(scm::math::vec2i(quad_node->data.get_flip_x() ? -1 : 1, quad_node->data.get_flip_y() ? -1 : 1));
    UniformValue opacity(quad_node->data.get_opacity());

    auto width(pipe.current_viewstate().target->get_width());
    auto height(pipe.current_viewstate().target->get_height());

    UniformValue size(scm::math::vec2f(1.0 * quad_node->data.get_size().x / width, 1.0 * quad_node->data.get_size().y / height));

    UniformValue offset(scm::math::vec2f((2.0 * quad_node->data.get_offset().x + quad_node->data.get_anchor().x * (width - quad_node->data.get_size().x)) / width,
                                         (2.0 * quad_node->data.get_offset().y + quad_node->data.get_anchor().y * (height - quad_node->data.get_size().y)) / height));

    auto const& ctx(pipe.get_context());

    pass.shader()->apply_uniform(ctx, "gua_in_texture", tex);
    pass.shader()->apply_uniform(ctx, "flip", flip);
    pass.shader()->apply_uniform(ctx, "size", size);
    pass.shader()->apply_uniform(ctx, "offset", offset);
    pass.shader()->apply_uniform(ctx, "opacity", opacity);

    if(render_multiview) {
        pipe.draw_quad_instanced();
    } else {
        pipe.draw_quad();
    }

}

void render_quads(PipelinePass& pass, PipelinePassDescription const&, Pipeline& pipe, bool render_multiview)
{
    auto& scene = *pipe.current_viewstate().scene;
    for(auto const& node : scene.nodes[std::type_index(typeid(node::TexturedScreenSpaceQuadNode))])
    {
        auto quad_node(reinterpret_cast<node::TexturedScreenSpaceQuadNode*>(node));
        render_node(pass, quad_node, pipe, render_multiview);
    }
}

////////////////////////////////////////////////////////////////////////////////

TexturedScreenSpaceQuadPassDescription::TexturedScreenSpaceQuadPassDescription() : PipelinePassDescription()
{
    vertex_shader_ = "shaders/textured_screen_space_quad.vert";
    fragment_shader_ = "shaders/textured_screen_space_quad.frag";
    private_.name_ = "TexturedScreenSpaceQuadPass";

    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = true;
    private_.rendermode_ = RenderMode::Callback;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(false, false));

    private_.blend_state_desc_ = boost::make_optional(
        scm::gl::blend_state_desc(scm::gl::blend_ops(true, scm::gl::FUNC_SRC_ALPHA, scm::gl::FUNC_ONE_MINUS_SRC_ALPHA, scm::gl::FUNC_SRC_ALPHA, scm::gl::FUNC_ONE_MINUS_SRC_ALPHA)));
    private_.process_ = render_quads;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> TexturedScreenSpaceQuadPassDescription::make_copy() const { return std::make_shared<TexturedScreenSpaceQuadPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass TexturedScreenSpaceQuadPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) { return PipelinePass{*this, ctx, substitution_map}; }

} // namespace gua
