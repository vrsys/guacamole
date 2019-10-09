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

#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

SSAOPassDescription::SSAOPassDescription() : PipelinePassDescription()
{
    vertex_shader_ = "shaders/common/fullscreen_quad.vert";
    fragment_shader_ = "shaders/ssao.frag";

    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = true;
    private_.rendermode_ = RenderMode::Quad;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(false, false));

    private_.blend_state_desc_ = boost::make_optional(
        scm::gl::blend_state_desc(scm::gl::blend_ops(true, scm::gl::FUNC_SRC_ALPHA, scm::gl::FUNC_ONE_MINUS_SRC_ALPHA, scm::gl::FUNC_SRC_ALPHA, scm::gl::FUNC_ONE_MINUS_SRC_ALPHA)));
    uniforms["gua_ssao_radius"] = 1.0f;
    uniforms["gua_ssao_intensity"] = 1.0f;
    uniforms["gua_ssao_falloff"] = 0.1f;
    uniforms["gua_noise_tex"] = std::string("gua_noise_texture");
}

////////////////////////////////////////////////////////////////////////////////

SSAOPassDescription& SSAOPassDescription::radius(float radius)
{
    uniforms["gua_ssao_radius"] = radius;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////

float SSAOPassDescription::radius() const
{
    auto uniform(uniforms.find("gua_ssao_radius"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

SSAOPassDescription& SSAOPassDescription::intensity(float intensity)
{
    uniforms["gua_ssao_intensity"] = intensity;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////

float SSAOPassDescription::intensity() const
{
    auto uniform(uniforms.find("gua_ssao_intensity"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

SSAOPassDescription& SSAOPassDescription::falloff(float falloff)
{
    uniforms["gua_ssao_falloff"] = falloff;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////

float SSAOPassDescription::falloff() const
{
    auto uniform(uniforms.find("gua_ssao_falloff"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> SSAOPassDescription::make_copy() const { return std::make_shared<SSAOPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass SSAOPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
