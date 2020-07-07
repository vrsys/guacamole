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
#include <gua/renderer/DepthCubeMapPass.hpp>

#include <gua/renderer/DepthCubeMapRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/Resources.hpp>

namespace gua
{
DepthCubeMapPassDesciption::DepthCubeMapPassDesciption() : PipelinePassDescription()
{
    vertex_shader_ = "resources/shaders/common/quad.vert";
    fragment_shader_ = "resources/shaders/textured_quad.frag";
    private_.name_ = "DepthCubeMapPass";

    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = false;
    private_.enable_for_shadows_ = false;
    private_.rendermode_ = RenderMode::Custom;

    private_.rasterizer_state_desc_ = boost::make_optional(scm::gl::rasterizer_state_desc(scm::gl::FILL_SOLID, scm::gl::CULL_NONE));

    // depth_stencil_state_ = boost::make_optional(
    //   scm::gl::depth_stencil_state_desc(
    //     true, true, scm::gl::COMPARISON_LESS, true, 1, 0,
    //     scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    //   )
    // );

    private_.depth_stencil_state_desc_ =
        boost::make_optional(scm::gl::depth_stencil_state_desc(false, false, scm::gl::COMPARISON_LESS, true, 0xFF, 0x00, scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)));

    // rasterizer_state_ = boost::make_optional(scm::gl::rasterizer_state_desc(
    //       scm::gl::FILL_SOLID, scm::gl::CULL_FRONT));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> DepthCubeMapPassDesciption::make_copy() const { return std::make_shared<DepthCubeMapPassDesciption>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass DepthCubeMapPassDesciption::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    auto renderer = std::make_shared<DepthCubeMapRenderer>();
    renderer->set_global_substitution_map(substitution_map);
    renderer->create_state_objects(ctx);

    private_.process_ = [renderer](PipelinePass& pass, PipelinePassDescription const& desc, Pipeline& pipe, bool render_multiview) {
        pipe.get_context().render_context->set_depth_stencil_state(pass.depth_stencil_state());
        pipe.get_context().render_context->set_rasterizer_state(pass.rasterizer_state());
        renderer->render(pipe, desc);
    };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
