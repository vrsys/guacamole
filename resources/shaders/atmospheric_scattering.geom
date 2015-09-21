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

@include "shaders/common/header.glsl"

layout(triangles, invocations = 6) in;
layout(triangle_strip, max_vertices = 24) out;

in vec3 varying_position[];
in vec2 varying_texcoord[];

out flat int texlayer;
out vec2 texcoords;

void main() {
  texlayer = gl_InvocationID;
  gl_Layer = texlayer;

  for(int i = 0; i < 3; ++i) {
    gl_Position = vec4(varying_position[i], 1);
    texcoords = varying_texcoord[i];
    EmitVertex();
  }
  EndPrimitive();
}
