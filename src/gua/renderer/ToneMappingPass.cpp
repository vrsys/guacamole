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
#include <gua/renderer/ToneMappingPass.hpp>

#include <gua/renderer/Pipeline.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

ToneMappingPassDescription::ToneMappingPassDescription() : PipelinePassDescription()
{
    vertex_shader_ = "shaders/common/fullscreen_quad.vert";
    fragment_shader_ = "shaders/tone_mapping.frag";
    private_.name_ = "ToneMappingPass";

    private_.needs_color_buffer_as_input_ = true;
    private_.writes_only_color_buffer_ = true;
    private_.rendermode_ = RenderMode::Quad;
    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(false, false));
    uniforms["gua_tone_mapping_exposure"] = 1.0f;
    uniforms["gua_tone_mapping_operator"] = static_cast<int>(Method::LINEAR);
}

////////////////////////////////////////////////////////////////////////////////

ToneMappingPassDescription& ToneMappingPassDescription::exposure(float e)
{
    uniforms["gua_tone_mapping_exposure"] = e;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////

float ToneMappingPassDescription::exposure() const
{
    auto uniform(uniforms.find("gua_tone_mapping_exposure"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

ToneMappingPassDescription& ToneMappingPassDescription::method(ToneMappingPassDescription::Method m)
{
    uniforms["gua_tone_mapping_operator"] = static_cast<int>(m);
    return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> ToneMappingPassDescription::make_copy() const { return std::make_shared<ToneMappingPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

ToneMappingPassDescription::Method ToneMappingPassDescription::method() const
{
    auto uniform(uniforms.find("gua_tone_mapping_operator"));
    return ToneMappingPassDescription::Method(boost::get<int>(uniform->second.data));
}

} // namespace gua
