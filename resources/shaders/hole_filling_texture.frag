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
uniform uvec2 hole_filling_texture;
uniform uvec2 color_buffer;
uniform uvec2 depth_buffer;

in vec2 gua_quad_coords;

layout(pixel_center_integer) in vec4 gl_FragCoord;

// write output
layout(location=0) out vec4 result;

void main() {
  vec4 samples[4*4];
  result = vec4(0);

  if (current_level == 0) {
    const ivec2 max_res = textureSize(sampler2D(color_buffer), 0);
    for (int x=0; x<4; ++x) {
      for (int y=0; y<4; ++y) {
        const ivec2 pos = clamp(ivec2(gl_FragCoord.xy*2) + ivec2(x-4/2+1, y-4/2+1), ivec2(0), max_res-1);
        samples[x+y*4].rgb = texelFetch(sampler2D(color_buffer), pos, 0).rgb;
        samples[x+y*4].a = texelFetch(sampler2D(depth_buffer), pos, 0).r;
      }
    }
  } else {
    const ivec2 max_res = textureSize(sampler2D(hole_filling_texture), current_level-1);
    for (int x=0; x<4; ++x) {
      for (int y=0; y<4; ++y) {
        const ivec2 pos = clamp(ivec2(gl_FragCoord.xy*2) + ivec2(x-4/2+1, y-4/2+1), ivec2(0), max_res-1);
        samples[x+y*4] = texelFetch(sampler2D(hole_filling_texture), pos, current_level-1);
      }
    }
  }

  // count number of hole pixels
  int hole_count = 0;

  for (int i=0; i<4*4; ++i) {
    if (samples[i].a == 1.0) ++hole_count;
  }

  // calculate average depth of none hole pixels
  if (hole_count < 4*4) {
    float average_depth = 0;
    for (int i=0; i<4*4; ++i) {
      average_depth += samples[i].a;
    }

    average_depth = (average_depth-hole_count) / (4*4-hole_count);

    float max_depth = 1;
    float weight = 0;
    float weights[16] = {0.4, 0.9, 0.9, 0.4,
                         0.9, 1.8, 1.8, 0.9,
                         0.9, 1.8, 1.8, 0.9,
                         0.4, 0.9, 0.9, 0.4,};
    for (int i=0; i<4*4; ++i) {
      // calculate average color of all none hole pixels with a depth larger or equal to average
      if (samples[i].a != 1.0 && samples[i].a > average_depth-0.000001) {
        max_depth = min(max_depth, samples[i].a);
        result.rgb += samples[i].rgb * weights[i];
        weight += weights[i];
      }
    }

    // return color and average depth
    if (weight > 0) {
      result /= weight;
    }

    result.a = max_depth;
  } else {
    result = vec4(0, 0, 0, 1);
  }
}
