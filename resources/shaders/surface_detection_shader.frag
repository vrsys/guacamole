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
@include "gbuffer_warp_modes.glsl"

uniform uvec2 depth_buffer;
uniform uvec2 surface_detection_buffer;
uniform int   current_level;

in vec2 gua_quad_coords;

layout(pixel_center_integer) in vec4 gl_FragCoord;

// write output
#if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD
  layout(location=0) out vec3 result;
#else
  layout(location=0) out uint result;
#endif

@include "shaders/warp_grid_bits.glsl"

uint is_on_line(float a, float b, float c, float d) {
  return int(abs(a-2*b+c) < @split_threshold@ && abs(b-2*c+d) < @split_threshold@);
}

uint is_on_line(float a, float b, float c) {
  return int(abs(a-2*b+c) < @split_threshold@);
}

void main() {

  // ---------------------------------------------------------------------------
  #if WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION
  // ---------------------------------------------------------------------------

  if (current_level == 0) {

    //    d0 d1
    //    |   |
    // d2-d3-d4-d5
    //    |   |
    // d6-d7-d8-d9
    //    |   |
    //   d10 d11

    const ivec2 res = textureSize(sampler2D(depth_buffer), 0) - 1;

    const float d0  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  2))), 0).x;
    const float d1  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  2))), 0).x;

    const float d2  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1,  1))), 0).x;
    const float d3  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  1))), 0).x;
    const float d4  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  1))), 0).x;
    const float d5  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2,  1))), 0).x;

    const float d6  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1,  0))), 0).x;
    const float d7  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  0))), 0).x;
    const float d8  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  0))), 0).x;
    const float d9  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2,  0))), 0).x;

    const float d10 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0, -1))), 0).x;
    const float d11 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1, -1))), 0).x;

    const uint is_surface = is_on_line(d0, d3, d7, d10)
                          & is_on_line(d1, d4, d8, d11)
                          & is_on_line(d2, d3, d4, d5)
                          & is_on_line(d6, d7, d8, d9);

    const uint merge_type = is_surface << BIT_MERGE_TYPE;

    result = is_surface | merge_type;

  } else {

    // s0-s1
    // |   |
    // s2-s3

    uint s0 = texelFetchOffset(usampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 1)).x;
    uint s1 = texelFetchOffset(usampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 1)).x;
    uint s2 = texelFetchOffset(usampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 0)).x;
    uint s3 = texelFetchOffset(usampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 0)).x;

    // if each child has been a connected surface, the parent is as a surface as well
    const uint is_surface = s0 & s1 & s2 & s3;
    const uint merge_type = is_surface << BIT_MERGE_TYPE;

    result = is_surface | merge_type;
  }

  // ---------------------------------------------------------------------------
  #elif WARP_MODE == WARP_MODE_GRID_ADVANCED_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION
  // ---------------------------------------------------------------------------

  if (current_level == 0) {

    // d0  d1   d2   d3
    //   \  |    |  /
    // d4--d5-- d6-- d7
    //      |    |
    // d8--d9--d10--d11
    //   /  |    |  \
    // d12 d13  d14 d15

    const ivec2 res = textureSize(sampler2D(depth_buffer), 0) - 1;

    const float d0  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1,  2))), 0).x;
    const float d1  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  2))), 0).x;
    const float d2  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  2))), 0).x;
    const float d3  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2,  2))), 0).x;

    const float d4  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1,  1))), 0).x;
    const float d5  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  1))), 0).x;
    const float d6  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  1))), 0).x;
    const float d7  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2,  1))), 0).x;

    const float d8  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1,  0))), 0).x;
    const float d9  = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0,  0))), 0).x;
    const float d10 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1,  0))), 0).x;
    const float d11 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2,  0))), 0).x;

    const float d12 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2(-1, -1))), 0).x;
    const float d13 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 0, -1))), 0).x;
    const float d14 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 1, -1))), 0).x;
    const float d15 = texelFetch(sampler2D(depth_buffer), max(ivec2(0), min(res, ivec2(gl_FragCoord.xy*2) + ivec2( 2, -1))), 0).x;

    // check for horizontal and vertical continuity
    const uint t = is_on_line(d1, d5, d9)  & is_on_line(d2, d6,  d10);
    const uint r = is_on_line(d5, d6, d7)  & is_on_line(d9, d10, d11);
    const uint b = is_on_line(d5, d9, d13) & is_on_line(d6, d10, d14);
    const uint l = is_on_line(d4, d5, d6)  & is_on_line(d8, d9,  d10);

    // check for diagonal continuity
    const uint tl = is_on_line(d0,  d5,  d10);
    const uint tr = is_on_line(d3,  d6,  d9);
    const uint bl = is_on_line(d12, d9,  d6);
    const uint br = is_on_line(d5,  d10, d15);

    // if the patch is connected on two othogonal sides, it represents a surface
    const uint is_surface = (t & r) | (r & b) | (b & l) | (l & t);
    const uint continuous = (t  << BIT_CONTINUOUS_T)
                          | (r  << BIT_CONTINUOUS_R)
                          | (b  << BIT_CONTINUOUS_B)
                          | (l  << BIT_CONTINUOUS_L)
                          | (tl << BIT_CONTINUOUS_TL)
                          | (tr << BIT_CONTINUOUS_TR)
                          | (bl << BIT_CONTINUOUS_BL)
                          | (br << BIT_CONTINUOUS_BR);

    const uint merge_type = is_surface << BIT_MERGE_TYPE;

    // store all continuities

    result = is_surface | merge_type | continuous;

    #if WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION
      if (merge_type == MERGE_NONE) {
        if (t == 1 || b == 1) {
          result = MERGE_LR | continuous;
        } else if (r == 1 || l == 1) {
          result = MERGE_TB | continuous;
        } else if ((is_on_line(d4, d5, d6) | is_on_line(d5, d6, d7)) == 1) {
          result = MERGE_T | continuous;
        } else if ((is_on_line(d8, d9, d10) | is_on_line(d9, d10, d11)) == 1) {
          result = MERGE_B | continuous;
        } else if ((is_on_line(d1, d5, d9) | is_on_line(d5, d9, d13)) == 1) {
          result = MERGE_L | continuous;
        } else if ((is_on_line(d2, d6, d10) | is_on_line(d6, d10, d14)) == 1) {
          result = MERGE_R | continuous;
        }
      }
    #endif

  } else {

    // s0-s1
    // |   |
    // s2-s3

    const uint s0 = texelFetchOffset(usampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 1)).x;
    const uint s1 = texelFetchOffset(usampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 1)).x;
    const uint s2 = texelFetchOffset(usampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 0)).x;
    const uint s3 = texelFetchOffset(usampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 0)).x;

    // check for internal continuity
    const uint internal_continuity = (s0 >> BIT_CONTINUOUS_R)
                                   & (s0 >> BIT_CONTINUOUS_B)
                                   & (s3 >> BIT_CONTINUOUS_T)
                                   & (s3 >> BIT_CONTINUOUS_L) & 1;

    // if any child is no complete surface, the parent is neither
    const uint is_surface = s0 & s1 & s2 & s3 & internal_continuity;
    const uint merge_type = is_surface << BIT_MERGE_TYPE;

    // check for horizontal and vertical continuity
    const uint t = (s0 >> BIT_CONTINUOUS_T) & (s1 >> BIT_CONTINUOUS_T) & 1;
    const uint r = (s1 >> BIT_CONTINUOUS_R) & (s3 >> BIT_CONTINUOUS_R) & 1;
    const uint b = (s2 >> BIT_CONTINUOUS_B) & (s3 >> BIT_CONTINUOUS_B) & 1;
    const uint l = (s0 >> BIT_CONTINUOUS_L) & (s2 >> BIT_CONTINUOUS_L) & 1;

    // check for diagonal continuity
    const uint tl = (s0 >> BIT_CONTINUOUS_TL) & 1;
    const uint tr = (s1 >> BIT_CONTINUOUS_TR) & 1;
    const uint bl = (s2 >> BIT_CONTINUOUS_BL) & 1;
    const uint br = (s3 >> BIT_CONTINUOUS_BR) & 1;

    // check for external continuity
    const uint continuous = (t  << BIT_CONTINUOUS_T)
                          | (r  << BIT_CONTINUOUS_R)
                          | (b  << BIT_CONTINUOUS_B)
                          | (l  << BIT_CONTINUOUS_L)
                          | (tl << BIT_CONTINUOUS_TL)
                          | (tr << BIT_CONTINUOUS_TR)
                          | (bl << BIT_CONTINUOUS_BL)
                          | (br << BIT_CONTINUOUS_BR);

    result = is_surface | merge_type | continuous;

    #if WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION
      if (merge_type == MERGE_NONE) {

        const uint merge_s0 = s0 & 1;
        const uint merge_s1 = s1 & 1;
        const uint merge_s2 = s2 & 1;
        const uint merge_s3 = s3 & 1;

        const uint merge_l = (s0 >> BIT_CONTINUOUS_B) & merge_s0 & merge_s2;
        const uint merge_r = (s1 >> BIT_CONTINUOUS_B) & merge_s1 & merge_s3;
        const uint merge_t = (s0 >> BIT_CONTINUOUS_R) & merge_s0 & merge_s1;
        const uint merge_b = (s2 >> BIT_CONTINUOUS_R) & merge_s2 & merge_s3;

        if (merge_l == 1) {
          if (merge_r == 1) {
            result = MERGE_LR | continuous;
          } else {
            result = MERGE_L | continuous;
          }
        } else if (merge_t == 1) {
          if (merge_b == 1) {
            result = MERGE_TB | continuous;
          } else {
            result = MERGE_T | continuous;
          }
        } else if (merge_r == 1) {
          result = MERGE_R | continuous;
        } else if (merge_b == 1) {
          result = MERGE_B | continuous;
        }
      }

    #endif
  }

  // ---------------------------------------------------------------------------
  #else // DEPTH_THRESHOLD -----------------------------------------------------
  // ---------------------------------------------------------------------------


  if (current_level == 0) {
    const float d0 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(0, 0)).x;
    const float d1 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(0, 1)).x;
    const float d2 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(1, 0)).x;
    const float d3 = texelFetchOffset(sampler2D(depth_buffer), ivec2(gl_FragCoord.xy*2), 0, ivec2(1, 1)).x;

    const float min_0 = min(d0, d1);
    const float min_1 = min(d2, d3);

    const float max_0 = max(d0, d1);
    const float max_1 = max(d2, d3);

    result.yz = vec2(min(min_0, min_1), max(max_0, max_1));
    result.x = abs(result.y - result.z) > @split_threshold@ ? 0 : 1;

  } else {
    const vec3 sample_0 = texelFetchOffset(sampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 0)).xyz;
    const vec3 sample_1 = texelFetchOffset(sampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 0)).xyz;
    const vec3 sample_2 = texelFetchOffset(sampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(0, 1)).xyz;
    const vec3 sample_3 = texelFetchOffset(sampler2D(surface_detection_buffer), ivec2(gl_FragCoord.xy*2), current_level-1, ivec2(1, 1)).xyz;

    result.x = min(min(sample_0.x, sample_1.x), min(sample_2.x, sample_3.x));

    if (result.x != 0) {
      const float min_0 = min(sample_0.y, sample_1.y);
      const float min_1 = min(sample_2.y, sample_3.y);

      const float max_0 = max(sample_0.z, sample_1.z);
      const float max_1 = max(sample_2.z, sample_3.z);

      result.yz = vec2(min(min_0, min_1), max(max_0, max_1));
      result.x = abs(result.y - result.z) > @split_threshold@ ? 0 : 1;
    }
  }

  #endif
}
