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
layout(line_strip, max_vertices = 20) out;

flat in uvec3 varying_position[];
flat out uint cellsize;

@include "shaders/warp_grid_bits.glsl"

void emit_quad(uvec2 offset, uint size) {
  vec2 vertex_position = vec2(varying_position[0].xy + offset) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + offset + vec2(size, 0)) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + offset + vec2(size, size)) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + offset + vec2(0, size)) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  vertex_position = vec2(varying_position[0].xy + offset) / gua_resolution * 2 - 1;
  gl_Position = vec4(vertex_position, 0, 1);
  EmitVertex();

  EndPrimitive();

}

void main() {

  uint level = varying_position[0].z >> BIT_CURRENT_LEVEL;
  cellsize = 1 << level;

  if ((varying_position[0].z & 1) > 0) {
    emit_quad(uvec2(0), cellsize);
  } else {
    const uvec2 offsets[4] = {uvec2(0), uvec2(1, 0),
                              uvec2(1), uvec2(0, 1)};
    for (int v=0; v<4; ++v) {
      emit_quad(offsets[v], 1);
    }
  }
}
