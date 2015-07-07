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
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_abuffer.glsl"

uniform int current_level;

in vec2 gua_quad_coords;

layout(pixel_center_integer) in vec4 gl_FragCoord;

// write output
layout(location=0) out uvec2 result;

void main() {

  if (current_level == 0) {
    const uint min_0 = imageLoad(abuf_min_depth, ivec2(gl_FragCoord.xy));
    const uint max_0 = imageLoad(abuf_max_depth, ivec2(gl_FragCoord.xy));

    result = uvec2(min_0, max_0);

  } else {
    const uvec2 sample_0 = texelFetchOffset(usampler2D(abuf_min_max_depth), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 0)).xy;
    const uvec2 sample_1 = texelFetchOffset(usampler2D(abuf_min_max_depth), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 0)).xy;
    const uvec2 sample_2 = texelFetchOffset(usampler2D(abuf_min_max_depth), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 1)).xy;
    const uvec2 sample_3 = texelFetchOffset(usampler2D(abuf_min_max_depth), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 1)).xy;

    result = uvec2(max(max(sample_0, sample_1), max(sample_2, sample_3)));
  }
}
