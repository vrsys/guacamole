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
@include "shaders/common/gua_camera_uniforms.glsl"

layout(points) in;
layout(line_strip, max_vertices = 5) out;

flat in ivec3 varying_position[];
flat out int cellsize;

void main() {

  cellsize = varying_position[0].z;

  vec2 vertex_position = vec2(varying_position[0].xy) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + vec2(cellsize, 0)) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + vec2(cellsize, cellsize)) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + vec2(0, cellsize)) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  EndPrimitive();
}
