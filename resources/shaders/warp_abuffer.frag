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

@include "common/header.glsl"
@include "common/gua_camera_uniforms.glsl"
@include "abuffer_warp_modes.glsl"

layout(pixel_center_integer) in vec4 gl_FragCoord;

// output
layout(location=0) out vec3 gua_out_color;

#define MAX_RAY_STEPS @warping_max_layers@

#if WARP_MODE == WARP_MODE_RAYCASTING

  uniform float gua_tone_mapping_exposure = 1.0;

  @include "common/gua_abuffer.glsl"
  @include "common/gua_tone_mapping.glsl"

  in vec2 gua_quad_coords;

  uniform mat4 warp_matrix;
  uniform uvec2 warped_depth_buffer;
  uniform uvec2 warped_color_buffer;

  bool get_ray(vec2 screen_space_pos, inout vec3 start, inout vec3 end, vec2 crop_depth) {
    vec2 coords = screen_space_pos * 2 - 1;
    float depth = texture2D(sampler2D(warped_depth_buffer), screen_space_pos).x;

    vec4 s = vec4(coords, -1, 1);
    vec4 e = vec4(coords, depth*2-1, 1);

    s = warp_matrix * s;
    e = warp_matrix * e;

    // remove flip version at the back
    // if (e.w < 0) {
    //   return false;
    // }

    // move start into original frustum
    if (s.w < 0) {
      s = mix(s, e, (s.z) / (s.z - e.z));
    }

    // perspective division
    s /= s.w;
    e /= e.w;

    // move end into original frustum
    // if (s.z > 1) {
    //   s = mix(s, e, (1 - s.z) / (e.z - s.z));
    // }


    // viewport clipping
    if (s.x > 1) {
      s.yz = mix(s.yz, e.yz, (1 - s.x) / (e.x - s.x));
      s.x = 1;
    }

    if (s.x < -1) {
      s.yz = mix(s.yz, e.yz, (-1 - s.x) / (e.x - s.x));
      s.x = -1;
    }

    if (s.y > 1) {
      s.xz = mix(s.xz, e.xz, (1 - s.y) / (e.y - s.y));
      s.y = 1;
    }

    if (s.y < -1) {
      s.xz = mix(s.xz, e.xz, (-1 - s.y) / (e.y - s.y));
      s.y = -1;
    }


    if (e.x > 1) {
      e.yz = mix(e.yz, s.yz, (1 - e.x) / (s.x - e.x));
      e.x = 1;
    }

    if (e.x < -1) {
      e.yz = mix(e.yz, s.yz, (-1 - e.x) / (s.x - e.x));
      e.x = -1;
    }

    if (e.y > 1) {
      e.xz = mix(e.xz, s.xz, (1 - e.y) / (s.y - e.y));
      e.y = 1;
    }

    if (e.y < -1) {
      e.xz = mix(e.xz, s.xz, (-1 - e.y) / (s.y - e.y));
      e.y = -1;
    }

    start = s.xyz * 0.5 + 0.5;
    end   = e.xyz * 0.5 + 0.5;

    if ((start.z < crop_depth.x && end.z < crop_depth.x) || (start.z > crop_depth.y && end.z > crop_depth.y) || 
        any(lessThan(start.xy, vec2(0))) || any(greaterThan(start.xy, vec2(1))) || crop_depth.x > crop_depth.y) {
      // invalid ray if either
      // 
      // - there is no transparent surface visible at all
      return false;
    }

    if (start.z < crop_depth.x) {
      start.xy = mix(end.xy, start.xy, (crop_depth.x - end.z) / (start.z - end.z));
      start.z = crop_depth.x;
    }

    if (end.z > crop_depth.y) {
      end.xy = mix(end.xy, start.xy, (crop_depth.y - end.z) / (start.z - end.z));
      end.z = crop_depth.y;
    }


    return true;

  }

  void abuf_mix_frag(vec4 frag_color, inout vec4 color) {
    frag_color.rgb *= frag_color.a;
    color += mix(frag_color, vec4(0.0), color.a);
  }

  vec3 heat(float v) {
    float value = 1.0-v;
    return (0.5+0.5*smoothstep(0.0, 0.1, value))*vec3(
      smoothstep(0.5, 0.3, value),
      value < 0.3 ? smoothstep(0.0, 0.3, value) : smoothstep(1.0, 0.6, value),
      smoothstep(0.4, 0.6, value)
    );
  }

  vec2 get_min_max_depth(ivec2 pos, int level) {
    uvec2 min_max_depth = texelFetch(usampler2D(abuf_min_max_depth), min(pos, textureSize(usampler2D(abuf_min_max_depth), level-1)-1), level-1).xy;
    float min_depth = 1-unpack_depth(min_max_depth.x);
    float max_depth =   unpack_depth(min_max_depth.y);
    return vec2(min_depth, max_depth);
  }

  void draw_debug_views() {

    const int preview_scale = 3;
    vec2 preview_coords = gua_quad_coords*preview_scale-vec2(0.09, 0.12);
    
    // center ray preview
    #if 1
      // draw mini version of depth buffer
      if (preview_coords.x < 1 && preview_coords.y < 1 && preview_coords.x > 0 && preview_coords.y > 0) {
        gua_out_color = vec3(get_min_max_depth(ivec2(preview_coords*gua_resolution)/2, 1).x);
      }

      {
        const int max_level = textureQueryLevels(usampler2D(abuf_min_max_depth));
        const vec2 total_min_max_depth = get_min_max_depth(ivec2(0), max_level);

        if (total_min_max_depth.x > total_min_max_depth.y) {
          return;
        }

        vec2 ref = preview_coords*gua_resolution;
        vec3 s, e;
        if (!get_ray(vec2(0.5), s, e, total_min_max_depth)) {
          return;
        }

        s.xy = vec2(gua_resolution) * s.xy;
        e.xy = vec2(gua_resolution) * e.xy;

        int sample_count = 0;

              vec2 pos = s.xy;
        const vec3 dir = e-s;

        const vec2 signs = vec2(s.x <= e.x ? 1 : -1, s.y <= e.y ? 1 : -1);
        const vec2 flip  = vec2(s.x <= e.x ? 1 :  0, s.y <= e.y ? 1 :  0);

        int current_level = min(max_level, int(log2(max(abs(dir.x), abs(dir.y))+1)+1));

        while(++sample_count < (MAX_RAY_STEPS+1)) {

          const float cell_size   = 1<<current_level;
          const vec2  cell_origin = cell_size * mix(ceil(pos/cell_size-1), floor(pos/cell_size), flip);

          bool inside = false;
          if (all(lessThan(ref-cell_origin, vec2(cell_size))) && all(greaterThanEqual(ref-cell_origin, vec2(0)))) {
            inside = true;
          }

          const vec2 min_max_depth = get_min_max_depth(ivec2(cell_origin.xy/cell_size), current_level);
          const vec2 corner_in_ray_direction = cell_origin + flip*cell_size;
          const vec2 t = (corner_in_ray_direction - pos) / dir.xy;

          vec2 d_range;
          vec2 new_pos;

          if (t.x < t.y) {
            new_pos = vec2(cell_origin.x + flip.x*cell_size, pos.y + dir.y*t.x);
            d_range = s.z + dir.z / dir.x * vec2(pos.x-s.x, new_pos.x-s.x);

          } else {
            new_pos = vec2(pos.x + dir.x*t.y, cell_origin.y + flip.y*cell_size);
            d_range = s.z + dir.z / dir.y * vec2(pos.y-s.y, new_pos.y-s.y);
          }

          const bool at_end = any(greaterThan((new_pos.xy - e.xy)*signs, vec2(0)));

          if (at_end) {
            d_range.y = e.z;
          }

          const bool intersects = !(d_range.x > min_max_depth.y || min_max_depth.x > d_range.y);


          if (intersects) {
            if (inside) {
              gua_out_color = mix(vec3(min_max_depth.x, 0, 0), gua_out_color, 0.9);
            }

            if (current_level == 0) {
              // check abuffer
            }

          } else {
            if (inside) {
              gua_out_color = mix(vec3(0, min_max_depth.x, 0), gua_out_color, 0.7);
            }
          }
        
          if (!intersects || current_level == 0) {
            pos = new_pos;

            if (any(equal(mod(pos / cell_size, 2), vec2(0)))) {
              current_level = min(current_level+1, max_level);
            }
          } else {
            --current_level;
          }

          if (!intersects && at_end) {
            break;
          }


        }
      }

      // draw line
      {
        vec2 cur_pos = preview_coords;
        vec3 s, e;
        const int max_level = textureQueryLevels(usampler2D(abuf_min_max_depth));
        const vec2 total_min_max_depth = get_min_max_depth(ivec2(0), max_level);
        get_ray(vec2(0.5), s, e, total_min_max_depth);
        if(abs((e.y-s.y)*cur_pos.x - (e.x-s.x)*cur_pos.y + e.x*s.y - e.y*s.x) / (length(s.xy-e.xy)+0.00001) < 0.002
           && length(e.xy-cur_pos.xy) + length(s.xy-cur_pos.xy) - 0.002 < length(s.xy-e.xy)) {

          gua_out_color = mix(vec3(1, 0, 0), vec3(0, 1, 0), length(e.xy-cur_pos.xy)/(length(s.xy-e.xy)+0.00001));
        }
      }


      // draw center dot
      if (length(vec2(0.5) - gua_quad_coords) < 0.001) {
        gua_out_color = vec3(1, 0, 1);
      }

      preview_coords -= vec2(0, 1.1); 
    #endif
    
  }

  void main() {

    vec4 color = vec4(0);

    const int max_level = textureQueryLevels(usampler2D(abuf_min_max_depth));
    const vec2 total_min_max_depth = get_min_max_depth(ivec2(0), max_level);

    int sample_count = 0;
    int perform_ray_casting = 1;

    vec3 s, e;
    if (!get_ray(gua_quad_coords, s, e, total_min_max_depth)) {
      // skip raycasting if invalid ray was generated
      sample_count = MAX_RAY_STEPS+1;
      perform_ray_casting = 0;
    }

    s.xy = vec2(gua_resolution) * s.xy;
    e.xy = vec2(gua_resolution) * e.xy;

          vec2 pos = s.xy;
    const vec3 dir = e-s + 0.000001;

    const vec2 signs = vec2(s.x <= e.x ? 1 : -1, s.y <= e.y ? 1 : -1);
    const vec2 flip  = vec2(s.x <= e.x ? 1 :  0, s.y <= e.y ? 1 :  0);

    #if @adaptive_entry_level@ == 1
      int current_level = min(max_level, int(log2(max(abs(dir.x), abs(dir.y))+1)));
    #else
      int current_level = max_level;
    #endif

    while(++sample_count < (MAX_RAY_STEPS+1)) {
      const float cell_size   = 1<<current_level;
      const vec2  cell_origin = cell_size * mix(ceil(pos/cell_size-1), floor(pos/cell_size), flip);
      const vec2  min_max_depth = get_min_max_depth(ivec2(cell_origin.xy/max(2, cell_size)), max(1, current_level));
      const vec2  corner_in_ray_direction = cell_origin + flip*cell_size;
      const vec2  t = (corner_in_ray_direction - pos) / dir.xy;

      vec2 d_range;
      vec2 new_pos;

      if (t.x < t.y) {
        new_pos = vec2(cell_origin.x + flip.x*cell_size, pos.y + dir.y*t.x);
        d_range = s.z + dir.z / dir.x * vec2(pos.x-s.x, new_pos.x-s.x);

      } else {
        new_pos = vec2(pos.x + dir.x*t.y, cell_origin.y + flip.y*cell_size);
        d_range = s.z + dir.z / dir.y * vec2(pos.y-s.y, new_pos.y-s.y);
      }

      const bool at_end = any(greaterThan((new_pos.xy - e.xy)*signs, vec2(0)));

      if (at_end) {
        d_range.y = e.z;
      }

      const bool intersects = !(d_range.x > min_max_depth.y || min_max_depth.x > d_range.y);


      if (intersects && current_level == 0) {
        // check abuffer
        uvec2 frag = unpackUint2x32(frag_list[gua_resolution.x * int(cell_origin.y) + int(cell_origin.x)]);
        while (frag.x != 0) {

          float z = unpack_depth24(frag.y);
          const float thickness = 0.000;
          if (d_range.y + thickness > z && d_range.x - thickness <= z) {
            uvec4 data = frag_data[frag.x - abuf_list_offset];
            float frag_alpha = float(bitfieldExtract(frag.y, 0, 8)) / 255.0;
            vec3  frag_color = uintBitsToFloat(data.rgb);
            abuf_mix_frag(vec4(frag_color, frag_alpha), color);
          }

          frag = unpackUint2x32(frag_list[frag.x]);
        }

      }

      #if @debug_bounding_volumes@ == 1
        // draw debug hierachy
        if (current_level == 0 && intersects) {
          abuf_mix_frag(vec4(vec3(1, 0, 0), 1), color);
        }
        abuf_mix_frag(vec4(heat(1-float(current_level) / max_level), 0.05), color);
      #endif

    
      if (!intersects || current_level == 0) {
        pos = new_pos;

        if (any(equal(mod(pos / cell_size, 2), vec2(0)))) {
          current_level = min(current_level+1, max_level);
        }
      } else {
        --current_level;
      }


      if (!intersects && at_end) {
        break;
      }
    }


    #if @debug_sample_count@ == 1
      // draw debug sample count
      abuf_mix_frag(vec4(heat(float((sample_count-1)*perform_ray_casting) / MAX_RAY_STEPS), 0.5), color);
    #endif

    abuf_mix_frag(texture2D(sampler2D(warped_color_buffer), gua_quad_coords), color);
    gua_out_color = toneMap(color.rgb);

    #if @debug_sample_ray@ == 1
      draw_debug_views();
    #endif
  }

#endif
