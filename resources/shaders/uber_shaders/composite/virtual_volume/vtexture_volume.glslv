
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
#version 420 core

#extension GL_ARB_shading_language_include : require
#extension GL_NV_gpu_shader5 : enable

// uniforms
//#include <scm/data/common/header.glsl>
#include </shaders/uber_shaders/common/gua_camera_uniforms.glsl>

// input
layout(location=0) in vec3 gua_in_position;

void main() {
    gl_Position = vec4(gua_in_position, 1.0);
}

#if 0

#version 420 core

#extension GL_ARB_shading_language_include : require

#include </scm/gl_util/camera_block.glslh>
#include </scm/data/volume/volume_uniform_block.glslh>

precision highp float;

// attribute layout definitions ///////////////////////////////////////////////////////////////////
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_texcoord;

// input/output definitions ///////////////////////////////////////////////////////////////////////
out per_vertex {
    smooth vec4 ray_entry_os;
    smooth vec4 ray_entry_ts;
} vertex_out;

// uniform input definitions //////////////////////////////////////////////////////////////////////

// implementation /////////////////////////////////////////////////////////////////////////////////
void main()
{
    vertex_out.ray_entry_os = vec4(in_position, 1.0);
    vertex_out.ray_entry_ts = vec4(in_position, 1.0) * vec4(volume_data.scale_obj_to_tex.xyz, 1.0);
    gl_Position             = volume_data.mvp_matrix * vec4(in_position, 1.0);
    //gl_Position           = camera_transform.vp_matrix * vec4(in_position, 1.0);
}

#endif