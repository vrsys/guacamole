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
#include <gua/renderer/OcclusionCullingTriMeshPass.hpp>

#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/OcclusionCullingTriMeshRenderer.hpp>

#include <gua/renderer/Pipeline.hpp>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
#include <gua/renderer/VTResponsibility.hpp>
#endif

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

OcclusionCullingTriMeshPassDescription::OcclusionCullingTriMeshPassDescription() : PipelinePassDescription(), enable_depth_complexity_vis_(false), enable_culling_geometry_vis_(false)
{
    vertex_shader_ = "";   // "shaders/tri_mesh_shader.vert";
    fragment_shader_ = ""; // "shaders/tri_mesh_shader.frag";
    private_.name_ = "OcclusionCullingTriMeshPass";

    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = false;
    private_.enable_for_shadows_ = true;
    private_.rendermode_ = RenderMode::Custom;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(true, true, scm::gl::COMPARISON_LESS, true, 1, 0, scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> OcclusionCullingTriMeshPassDescription::make_copy() const { return std::make_shared<OcclusionCullingTriMeshPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass OcclusionCullingTriMeshPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    auto renderer = std::make_shared<OcclusionCullingTriMeshRenderer>(ctx, substitution_map);

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    pipeline_responsibilities_.push_back(std::make_shared<VTPreResponsibilityDescription>(*renderer.get()));
    pipeline_responsibilities_.push_back(std::make_shared<VTPostResponsibilityDescription>(*renderer.get()));
#endif

    private_.process_ = [renderer](PipelinePass& pass, PipelinePassDescription const& desc, Pipeline& pipe, bool render_multiview, bool use_hardware_mvr) {
        pipe.get_context().render_context->set_depth_stencil_state(pass.depth_stencil_state(), 1);
        renderer->render(pipe, desc);
    };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

///////////////////////////////////////////////////////////////////////////////////
void OcclusionCullingTriMeshPassDescription::set_enable_depth_complexity_vis(bool enable) {
    enable_depth_complexity_vis_ = enable;
}

///////////////////////////////////////////////////////////////////////////////////
bool OcclusionCullingTriMeshPassDescription::get_enable_depth_complexity_vis() const {
    return enable_depth_complexity_vis_;
}

///////////////////////////////////////////////////////////////////////////////////
void OcclusionCullingTriMeshPassDescription::set_enable_culling_geometry_vis(bool enable) {
    enable_culling_geometry_vis_ = enable;
}

///////////////////////////////////////////////////////////////////////////////////
bool OcclusionCullingTriMeshPassDescription::get_enable_culling_geometry_vis() const {
    return enable_culling_geometry_vis_;
}

} // namespace gua
