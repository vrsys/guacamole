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

uniform uvec2 depth_buffer;
uniform uvec2 min_max_depth_buffer;
uniform int   current_level;

in vec2 gua_quad_coords;

layout(pixel_center_integer) in vec4 gl_FragCoord;

// write outputs
layout(location=0) out vec2 gua_out_min_max;

void main() {
  if (current_level == 0) {
    float sample_0 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(0, 0)).r;
    float sample_1 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(0, 1)).r;
    float sample_2 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(1, 0)).r;
    float sample_3 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(1, 1)).r;

    float min_0 = min(sample_0, sample_1);
    float min_1 = min(sample_2, sample_3);

    float max_0 = max(sample_0, sample_1);
    float max_1 = max(sample_2, sample_3);

    gua_out_min_max = vec2(min(min_0, min_1), max(max_0, max_1));
  } else {
    vec2 sample_0 = texelFetchOffset(sampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 0)).rg;
    vec2 sample_1 = texelFetchOffset(sampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 0)).rg;
    vec2 sample_2 = texelFetchOffset(sampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 1)).rg;
    vec2 sample_3 = texelFetchOffset(sampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 1)).rg;

    float min_0 = min(sample_0.x, sample_1.x);
    float min_1 = min(sample_2.x, sample_3.x);

    float max_0 = max(sample_0.y, sample_1.y);
    float max_1 = max(sample_2.y, sample_3.y);

    gua_out_min_max = vec2(min(min_0, min_1), max(max_0, max_1));
  }
}
