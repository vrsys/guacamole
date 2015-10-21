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

uniform int current_level;
uniform uvec2 color_buffer;

in vec2 gua_quad_coords;

// layout(pixel_center_integer) in vec4 gl_FragCoord;

// write output
layout(location=0) out vec3 result;

void main() {
  const vec3 sample_0 = texelFetchOffset(sampler2D(color_buffer), ivec2(gl_FragCoord.xy*2), current_level, ivec2(0, 0)).rgb;
  const vec3 sample_1 = texelFetchOffset(sampler2D(color_buffer), ivec2(gl_FragCoord.xy*2), current_level, ivec2(1, 0)).rgb;
  const vec3 sample_2 = texelFetchOffset(sampler2D(color_buffer), ivec2(gl_FragCoord.xy*2), current_level, ivec2(0, 1)).rgb;
  const vec3 sample_3 = texelFetchOffset(sampler2D(color_buffer), ivec2(gl_FragCoord.xy*2), current_level, ivec2(1, 1)).rgb;

  result = (sample_0 + sample_1 + sample_2 + sample_3) / 4;

  // result = vec3(0, current_level*0.33, 1.0-current_level*0.33);
}
