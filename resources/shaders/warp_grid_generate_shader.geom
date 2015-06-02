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

layout(points) in;
layout(points, max_vertices = 4) out;

@include "shaders/common/gua_camera_uniforms.glsl"

in ivec3 varying_position[];

uniform uvec2 min_max_depth_buffer;
uniform int current_cellsize;

out ivec3 xfb_output;

void main() { 

  if (current_cellsize == varying_position[0].z) {
    const int new_cellsize = varying_position[0].z/2;
    const int layer = int(log2(new_cellsize));

    #if @generation_mode@ == 2 // DEPTH_THRESHOLD
    float is_surface = texelFetch(sampler2D(min_max_depth_buffer), varying_position[0].xy/varying_position[0].z, layer).x;
    if (is_surface == 0) {
    #else
    int is_surface = texelFetch(isampler2D(min_max_depth_buffer), varying_position[0].xy/varying_position[0].z, layer).x;
    if ((is_surface & 1) == 0) {
    #endif

      const vec2 offsets[4] = {ivec2(0), ivec2(new_cellsize, 0),
                               ivec2(new_cellsize), ivec2(0, new_cellsize)};

      for (int v=0; v<4; ++v) {
        xfb_output = ivec3(varying_position[0].xy + offsets[v], new_cellsize);
        EmitVertex();
        EndPrimitive();
      }
      

    } else {
      xfb_output = varying_position[0];
      EmitVertex(); 
      EndPrimitive();
    }

  } else {
    xfb_output = varying_position[0];
    EmitVertex(); 
    EndPrimitive();
  }
}
