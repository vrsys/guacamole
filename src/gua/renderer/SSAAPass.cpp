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
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////
SSAAPassDescription::SSAAPassDescription() : PipelinePassDescription()
{
    vertex_shader_ = "shaders/common/fullscreen_quad.vert";
    fragment_shader_ = "shaders/ssaa.frag";
    private_.needs_color_buffer_as_input_ = true;
    private_.writes_only_color_buffer_ = true;
    private_.rendermode_ = RenderMode::Quad;
    private_.name_ = "SSAAPassDescription";
    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(false, false));

    uniforms["gua_ssaa_mode"] = static_cast<int>(SSAAMode::FAST_FXAA);
    uniforms["gua_fxaa_quality_subpix"] = 0.75f;
    uniforms["gua_fxaa_edge_threshold"] = 0.125f;
    uniforms["gua_fxaa_threshold_min"] = 0.0625f;

    uniforms["gua_enable_pinhole_correction"] = false;
}

////////////////////////////////////////////////////////////////////////////////
SSAAPassDescription& SSAAPassDescription::mode(SSAAMode mode)
{
    uniforms["gua_ssaa_mode"] = static_cast<int>(mode);
    ;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
SSAAPassDescription::SSAAMode SSAAPassDescription::mode() const
{
    auto uniform(uniforms.find("gua_ssaa_mode"));
    return static_cast<SSAAPassDescription::SSAAMode>(boost::get<int>(uniform->second.data));
}

////////////////////////////////////////////////////////////////////////////////
SSAAPassDescription& SSAAPassDescription::fxaa_quality_subpix(float quality)
{
    uniforms["gua_fxaa_quality_subpix"] = quality;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
float SSAAPassDescription::fxaa_quality_subpix() const
{
    auto uniform(uniforms.find("gua_fxaa_quality_subpix"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
SSAAPassDescription& SSAAPassDescription::fxaa_edge_threshold(float threshold)
{
    uniforms["gua_fxaa_quality_subpix"] = threshold;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
float SSAAPassDescription::fxaa_edge_threshold() const
{
    auto uniform(uniforms.find("gua_fxaa_edge_threshold"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
SSAAPassDescription& SSAAPassDescription::fxaa_threshold_min(float minimum)
{
    uniforms["gua_fxaa_threshold_min"] = minimum;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
float SSAAPassDescription::fxaa_threshold_min() const
{
    auto uniform(uniforms.find("gua_fxaa_threshold_min"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
SSAAPassDescription& SSAAPassDescription::enable_pinhole_correction(bool enable)
{
    uniforms["gua_enable_pinhole_correction"] = enable;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool SSAAPassDescription::enable_pinhole_correction() const
{
    auto uniform(uniforms.find("gua_enable_pinhole_correction"));
    return boost::get<bool>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PipelinePassDescription> SSAAPassDescription::make_copy() const { return std::make_shared<SSAAPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////
PipelinePass SSAAPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
