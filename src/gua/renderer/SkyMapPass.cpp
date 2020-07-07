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
#include <gua/renderer/SkyMapPass.hpp>

#include <gua/renderer/SkyMapRenderer.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

SkyMapPassDescription::SkyMapPassDescription() : PipelinePassDescription()
{
    vertex_shader_ = "";   // "shaders/tri_mesh_shader.vert";
    fragment_shader_ = ""; // "shaders/tri_mesh_shader.frag";
    private_.name_ = "SkyMapPass";

    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = true;
    private_.enable_for_shadows_ = false;
    private_.rendermode_ = RenderMode::Custom;

    uniforms["light_direction"] = math::vec3f(0, -1, 0);
    uniforms["light_color"] = math::vec3f(0.65, 0.57, 0.475);
    uniforms["light_brightness"] = 15.f;
    uniforms["ground_color"] = math::vec3f(1.f);
    uniforms["rayleigh_factor"] = 2.5f;
    uniforms["mie_factor"] = 0.5f;
    uniforms["output_texture_name"] = std::string("");
}

////////////////////////////////////////////////////////////////////////////////
SkyMapPassDescription& SkyMapPassDescription::light_direction(math::vec3f const& light_direction)
{
    uniforms["light_direction"] = light_direction;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
math::vec3f SkyMapPassDescription::light_direction() const
{
    auto uniform(uniforms.find("light_direction"));
    return boost::get<math::vec3f>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
SkyMapPassDescription& SkyMapPassDescription::light_color(math::vec3f const& light_color)
{
    uniforms["light_color"] = light_color;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
math::vec3f SkyMapPassDescription::light_color() const
{
    auto uniform(uniforms.find("light_color"));
    return boost::get<math::vec3f>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
SkyMapPassDescription& SkyMapPassDescription::light_brightness(float light_brightness)
{
    uniforms["light_brightness"] = light_brightness;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
float SkyMapPassDescription::light_brightness() const
{
    auto uniform(uniforms.find("light_brightness"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
SkyMapPassDescription& SkyMapPassDescription::ground_color(math::vec3f const& ground_color)
{
    uniforms["ground_color"] = ground_color;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
math::vec3f SkyMapPassDescription::ground_color() const
{
    auto uniform(uniforms.find("ground_color"));
    return boost::get<math::vec3f>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
SkyMapPassDescription& SkyMapPassDescription::rayleigh_factor(float rayleigh_factor)
{
    uniforms["rayleigh_factor"] = rayleigh_factor;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
float SkyMapPassDescription::rayleigh_factor() const
{
    auto uniform(uniforms.find("rayleigh_factor"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
SkyMapPassDescription& SkyMapPassDescription::mie_factor(float mie_factor)
{
    uniforms["mie_factor"] = mie_factor;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
float SkyMapPassDescription::mie_factor() const
{
    auto uniform(uniforms.find("mie_factor"));
    return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
SkyMapPassDescription& SkyMapPassDescription::output_texture_name(std::string const& output_texture_name)
{
    uniforms["output_texture_name"] = output_texture_name;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
std::string SkyMapPassDescription::output_texture_name() const
{
    auto uniform(uniforms.find("output_texture_name"));
    return boost::get<std::string>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> SkyMapPassDescription::make_copy() const { return std::make_shared<SkyMapPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass SkyMapPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    auto renderer = std::make_shared<SkyMapRenderer>();

    private_.process_ = [renderer](PipelinePass& pass, PipelinePassDescription const& desc, Pipeline& pipe, bool render_multiview, bool use_hardware_mvr) { renderer->render_sky_map(pipe, desc); };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
