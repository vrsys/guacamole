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
#include <gua/skelanim/renderer/SkeletalAnimationPass.hpp>

// guacamole headers
#include <gua/skelanim/renderer/SkeletalAnimationRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

SkeletalAnimationPassDescription::SkeletalAnimationPassDescription() : PipelinePassDescription()
{
    vertex_shader_ = "";   // "shaders/tri_mesh_shader.vert";
    fragment_shader_ = ""; // "shaders/tri_mesh_shader.frag";
    private_.name_ = "SkeletalAnimationPass";

    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = false;
    private_.enable_for_shadows_ = true;
    private_.rendermode_ = RenderMode::Custom;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(true, true, scm::gl::COMPARISON_LESS, true, 1, 0, scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> SkeletalAnimationPassDescription::make_copy() const { return std::make_shared<SkeletalAnimationPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass SkeletalAnimationPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    auto renderer = std::make_shared<SkeletalAnimationRenderer>(ctx);
    renderer->set_global_substitution_map(substitution_map);
    renderer->create_state_objects(ctx);

    private_.process_ = [renderer](PipelinePass& pass, PipelinePassDescription const& desc, Pipeline& pipe, bool render_multiview, bool use_hardware_mvr) {
        pipe.get_context().render_context->set_depth_stencil_state(pass.depth_stencil_state(), 1);
        renderer->render(pipe, desc);
    };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
