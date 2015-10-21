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

  vec4 samples[16];
  result = vec4(0);

  if (current_level == 0) {
    for (int x=0; x<4; ++x) {
      for (int y=0; y<4; ++y) {
        samples[x+y*4].rgb = texelFetchOffset(sampler2D(color_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(x-1, y-1)).rgb;
        samples[x+y*4].a = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(x-1, y-1)).r;
      }
    }
  } else {
    for (int x=0; x<4; ++x) {
      for (int y=0; y<4; ++y) {
        samples[x+y*4] = texelFetchOffset(sampler2D(hole_filling_texture), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(x-1, y-1));
      }
    }
  }

  // count number of hole pixels
  // (if 0 return (0, 0, 0, 0) might be interesting)
  int hole_count = 0;

  for (int i=0; i<16; ++i) {
    if (samples[i].a == 1.0) ++hole_count;
  }

  // if (hole_count == 0) {
  //   result = vec4(0, 0, 0, 1);
  //   return;
  // }


  // calculate average depth of none hole pixels
  if (hole_count < 16) {
    float average_depth = 0;
    for (int i=0; i<16; ++i) {
      average_depth += samples[i].a;
    }

    average_depth = (average_depth-hole_count) / (16-hole_count);

    float max_depth = 0;
    int count = 0;
    for (int i=0; i<16; ++i) {
      // calculate average color of all none hole pixels with a depth larger or equal to average
      if (samples[i].a != 1.0 && samples[i].a > average_depth-0.000001) {
        max_depth = max(max_depth, samples[i].a);
        result += samples[i];
        ++count;
      }
    }

    // return color and average depth
    result /= count;
    result.a = max_depth;
  } else {
    result = vec4(0, 0, 0, 1);
  }
}
