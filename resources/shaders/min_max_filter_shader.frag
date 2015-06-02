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

// write output
#if @generation_mode@ == 2 // DEPTH_THRESHOLD
layout(location=0) out vec3 output_is_surface;
#else
layout(location=0) out int output_is_surface;
#endif

int is_on_line(float a, float b, float c, float d) {
  return int(abs(a-2*b+c) < @split_threshold@ && abs(b-2*c+d) < @split_threshold@);
}

int is_on_line(float a, float b, float c) {
  return int(abs(a-2*b+c) < @split_threshold@);
}

void main() {

  // ---------------------------------------------------------------------------
  #if @generation_mode@ == 0 // SURFACE_ESTIMATION -----------------------------
  // ---------------------------------------------------------------------------

  if (current_level == 0) {

    //    d0 d1
    //    |   |
    // d2-d3-d4-d5
    //    |   |  
    // d6-d7-d8-d9
    //    |   |
    //   d10 d11

    ivec2 res = textureSize(sampler2D(depth_buffer), 0) - 1;

    float d0  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  2))), 0).x;
    float d1  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  2))), 0).x;

    float d2  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1,  1))), 0).x;
    float d3  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  1))), 0).x;
    float d4  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  1))), 0).x;
    float d5  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2,  1))), 0).x;

    float d6  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1,  0))), 0).x;
    float d7  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  0))), 0).x;
    float d8  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  0))), 0).x;
    float d9  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2,  0))), 0).x;

    float d10 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0, -1))), 0).x;
    float d11 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1, -1))), 0).x;

    output_is_surface = is_on_line(d0, d3, d7, d10)
                      & is_on_line(d1, d4, d8, d11)
                      & is_on_line(d2, d3, d4, d5)
                      & is_on_line(d6, d7, d8, d9);

  } else {

    // s0-s1
    // |   | 
    // s2-s3

    int s0 = texelFetchOffset(isampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 1)).x;
    int s1 = texelFetchOffset(isampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 1)).x;
    int s2 = texelFetchOffset(isampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 0)).x;
    int s3 = texelFetchOffset(isampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 0)).x;

    output_is_surface = s0 & s1 & s2 & s3;
  }

  // ---------------------------------------------------------------------------
  #elif @generation_mode@ == 1 // ADAPTIVE_SURFACE_ESTIMATION ------------------
  // ---------------------------------------------------------------------------

  #define CONTINIOUS_T  1
  #define CONTINIOUS_R  2
  #define CONTINIOUS_B  3
  #define CONTINIOUS_L  4

  if (current_level == 0) {

    //    d0 d1
    //    |   |
    // d2-d3-d4-d5
    //    |   |  
    // d6-d7-d8-d9
    //    |   |
    //   d10 d11

    ivec2 res = textureSize(sampler2D(depth_buffer), 0) - 1;

    float d0  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  2))), 0).x;
    float d1  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  2))), 0).x;

    float d2  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1,  1))), 0).x;
    float d3  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  1))), 0).x;
    float d4  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  1))), 0).x;
    float d5  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2,  1))), 0).x;

    float d6  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1,  0))), 0).x;
    float d7  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  0))), 0).x;
    float d8  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  0))), 0).x;
    float d9  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2,  0))), 0).x;

    float d10 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0, -1))), 0).x;
    float d11 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1, -1))), 0).x;

    int t = is_on_line(d0, d3, d7)  & is_on_line(d1, d4, d8);
    int r = is_on_line(d3, d4, d5)  & is_on_line(d7, d8, d9);
    int b = is_on_line(d3, d7, d10) & is_on_line(d4, d8, d11);
    int l = is_on_line(d2, d3, d4)  & is_on_line(d6, d7, d8);


    int check1 = is_on_line(d0, d3, d7) & is_on_line(d1, d4, d8) & is_on_line(d3, d4, d5) & is_on_line(d7, d8, d9);
    int check2 = is_on_line(d0, d3, d7) & is_on_line(d1, d4, d8) & is_on_line(d2, d3, d4) & is_on_line(d6, d7, d8);
    int check3 = is_on_line(d3, d7, d10) & is_on_line(d4, d8, d11) & is_on_line(d3, d4, d5) & is_on_line(d7, d8, d9);
    int check4 = is_on_line(d3, d7, d10) & is_on_line(d4, d8, d11) & is_on_line(d2, d3, d4) & is_on_line(d6, d7, d8);

    int is_surface = check1 | check2 | check3 | check4;

    int continious = (t << CONTINIOUS_T)
                   | (r << CONTINIOUS_R)
                   | (b << CONTINIOUS_B)
                   | (l << CONTINIOUS_L);

    output_is_surface = is_surface | continious;

  } else {

    // s0-s1
    // |   | 
    // s2-s3

    int s0 = texelFetchOffset(isampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 1)).x;
    int s1 = texelFetchOffset(isampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 1)).x;
    int s2 = texelFetchOffset(isampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 0)).x;
    int s3 = texelFetchOffset(isampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 0)).x;

    // check for internal continuity
    int internal_continuity = (s0 >> CONTINIOUS_R)  
                            & (s0 >> CONTINIOUS_B) 
                            & (s3 >> CONTINIOUS_T)    
                            & (s3 >> CONTINIOUS_L) & 1;

    // if any child is no complete surface, the parent is neither
    int is_surface = s0 & s1 & s2 & s3 & internal_continuity;

    // propagate continuities
    int continious = (((s0 >> CONTINIOUS_T) & (s1 >> CONTINIOUS_T) & 1) << CONTINIOUS_T)    
                   | (((s1 >> CONTINIOUS_R) & (s3 >> CONTINIOUS_R) & 1) << CONTINIOUS_R)   
                   | (((s2 >> CONTINIOUS_B) & (s3 >> CONTINIOUS_B) & 1) << CONTINIOUS_B)
                   | (((s0 >> CONTINIOUS_L) & (s2 >> CONTINIOUS_L) & 1) << CONTINIOUS_L);

    output_is_surface = is_surface | continious;
  }

  // ---------------------------------------------------------------------------
  #else // DEPTH_THRESHOLD -----------------------------------------------------
  // ---------------------------------------------------------------------------


  if (current_level == 0) {
    float d0 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(0, 0)).x;
    float d1 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(0, 1)).x;
    float d2 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(1, 0)).x;
    float d3 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(1, 1)).x;

    float min_0 = min(d0, d1);
    float min_1 = min(d2, d3);

    float max_0 = max(d0, d1);
    float max_1 = max(d2, d3);

    output_is_surface.yz = vec2(min(min_0, min_1), max(max_0, max_1));
    output_is_surface.x = abs(output_is_surface.y - output_is_surface.z) > @split_threshold@ ? 0 : 1;

  } else {
    vec3 sample_0 = texelFetchOffset(sampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 0)).xyz;
    vec3 sample_1 = texelFetchOffset(sampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 0)).xyz;
    vec3 sample_2 = texelFetchOffset(sampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 1)).xyz;
    vec3 sample_3 = texelFetchOffset(sampler2D(min_max_depth_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 1)).xyz;

    output_is_surface.x = min(min(sample_0.x, sample_1.x), min(sample_2.x, sample_3.x));

    if (output_is_surface.x != 0) {
      float min_0 = min(sample_0.y, sample_1.y);
      float min_1 = min(sample_2.y, sample_3.y);

      float max_0 = max(sample_0.z, sample_1.z);
      float max_1 = max(sample_2.z, sample_3.z);

      output_is_surface.yz = vec2(min(min_0, min_1), max(max_0, max_1));
      output_is_surface.x = abs(output_is_surface.y - output_is_surface.z) > @split_threshold@ ? 0 : 1;
    }
  }

  #endif
}
