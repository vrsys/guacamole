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
#include <gua/renderer/FullscreenPass.hpp>

#include <gua/config.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ResourceFactory.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua
{
FullscreenPassDescription::FullscreenPassDescription() : PipelinePassDescription()
{
    vertex_shader_ = "shaders/common/fullscreen_quad.vert";
    fragment_shader_ = R"(
#version 440
// header
#extension GL_NV_bindless_texture  : require
#extension GL_ARB_bindless_texture : enable
#extension GL_NV_gpu_shader5       : enable

#ifdef GL_NV_shader_atomic_int64
#extension GL_NV_shader_atomic_int64 : enable
#endif

layout(location=0) out vec3 gua_out_color;
void main() {
  gua_out_color = vec4(1,0,0,0);
}
  )";

#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    fragment_shader_ = factory.prepare_shader(fragment_shader_, "FullscreenPass shader");
#else
    Resources::resolve_includes(fragment_shader_);
#endif

    fragment_shader_is_file_name_ = false;
    private_.needs_color_buffer_as_input_ = true;
    private_.writes_only_color_buffer_ = true;

    private_.rendermode_ = RenderMode::Quad;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(false, false));
}

////////////////////////////////////////////////////////////////////////////////

FullscreenPassDescription& FullscreenPassDescription::source(std::string const& source)
{
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    fragment_shader_ = factory.prepare_shader(source, "FullscreenPass shader");
#else
    fragment_shader_ = source;
    Resources::resolve_includes(fragment_shader_);
#endif
    fragment_shader_is_file_name_ = false;
    recompile_shaders_ = true;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& FullscreenPassDescription::source() const { return fragment_shader_; }

////////////////////////////////////////////////////////////////////////////////

FullscreenPassDescription& FullscreenPassDescription::source_file(std::string const& source_file)
{
    fragment_shader_ = source_file;
    fragment_shader_is_file_name_ = true;
    recompile_shaders_ = true;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& FullscreenPassDescription::source_file() const { return fragment_shader_; }

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> FullscreenPassDescription::make_copy() const { return std::make_shared<FullscreenPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass FullscreenPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
