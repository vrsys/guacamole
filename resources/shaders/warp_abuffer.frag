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
@include "hole_filling_modes.glsl"

layout(pixel_center_integer) in vec4 gl_FragCoord;

// output
layout(location=0) out vec3 gua_out_color;

#define MAX_RAY_STEPS @warping_max_layers@

uniform float gua_tone_mapping_exposure = 1.5;

@include "common/gua_abuffer.glsl"
@include "common/gua_tone_mapping.glsl"

in vec2 gua_quad_coords;

uniform mat4 inv_warp_matrix;
uniform mat4 warp_matrix;
uniform uvec2 warped_depth_buffer;
uniform uvec2 warped_color_buffer;
uniform uvec2 color_buffer;

bool get_ray(vec2 screen_space_pos, inout vec3 start, inout vec3 end, vec2 crop_depth) {
  vec2 coords = screen_space_pos * 2 - 1;
  float depth = texture2D(sampler2D(warped_depth_buffer), screen_space_pos).x;

  vec4 s = vec4(coords, -1, 1);
  vec4 e = vec4(coords, depth*2-1, 1);

  s = inv_warp_matrix * s;
  e = inv_warp_matrix * e;

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
    // invalid ray if it does not intersect with the abuffer contents at all
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
          if (d_range.x < min_max_depth.x) {
            pos = pos + dir.xy*(min_max_depth.x - d_range.x) / dir.z;
          }
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

vec4 hole_filling_inpaint() {
  float max_depth = 0;
  vec2  offset;

  for (int i=2; i<=5; ++i) {

    float radius = i*i;

    vec2 offsets[] = {          vec2(0, radius),
                        vec2(-radius, 0),   vec2(radius, 0),
                                vec2(0, -radius)};

    for (int i=0; i<offsets.length(); ++i) {
      float sample_depth = texture2D(sampler2D(warped_depth_buffer), gua_quad_coords + offsets[i]/gua_resolution).x;
      if (sample_depth > max_depth+0.01 && sample_depth < 1.0) {
        offset = offsets[i];
        max_depth = sample_depth;
      }
    }
  }
  if (max_depth > 0) {
    return texture2D(sampler2D(warped_color_buffer), gua_quad_coords + offset/gua_resolution);
  }

  return vec4(0, 0, 0, 1);
}

vec2 get_epipolar_direction() {
  vec4 epipol = warp_matrix * vec4(0, 0, -1, 0);
  vec2 epi_dir = vec2(0);

  if (epipol.w < 0) {
    epipol /= epipol.w;
    epipol.xy = epipol.xy*0.5 + 0.5;
    epi_dir = epipol.xy - gua_quad_coords;
  } else {
    epipol /= epipol.w;
    epipol.xy = epipol.xy*0.5 + 0.5;
    epi_dir = gua_quad_coords - epipol.xy;
  }

  return normalize(epi_dir);
}

vec4 hole_filling_epipolar_search() {
  vec2 boundary_pos = vec2(0);

  vec2 epi_dir = get_epipolar_direction();

  for (int i=1; i<=50; ++i) {
    boundary_pos = gua_quad_coords + i*epi_dir/gua_resolution;
    float sample_depth = texelFetch(sampler2D(warped_depth_buffer), ivec2(boundary_pos*gua_resolution), 0).x;

    if (sample_depth < 1.0) break;
  }

  return texelFetch(sampler2D(warped_color_buffer), ivec2(boundary_pos*gua_resolution),0);
}

vec4 hole_filling_epipolar_mirror() {
  vec2 boundary_pos = vec2(0);

  vec2 epi_dir = get_epipolar_direction();

  for (int i=1; i<=25; ++i) {
    boundary_pos = gua_quad_coords + i*epi_dir/gua_resolution;
    float sample_depth = texelFetch(sampler2D(warped_depth_buffer), ivec2(boundary_pos*gua_resolution), 0).x;

    if (sample_depth < 1.0) {
      sample_depth = texelFetch(sampler2D(warped_depth_buffer), ivec2((2*boundary_pos - gua_quad_coords)*gua_resolution), 0).x;
      if (sample_depth < 1.0) break;
    }
  }

  return texelFetch(sampler2D(warped_color_buffer), ivec2((2*boundary_pos - gua_quad_coords)*gua_resolution), 0);
}

vec4 hole_filling_blur() {
  int level = 5;
  // return textureLod(sampler2D(warped_color_buffer), gua_quad_coords, 1.0*level);
  return texelFetch(sampler2D(warped_color_buffer), ivec2(gua_quad_coords*gua_resolution)/(1<<level), level);
}

void main() {

  vec4 color = vec4(0);
  vec4 background_color = vec4(0, 0, 0, 1);


  // hole filling
  float depth = texture2D(sampler2D(warped_depth_buffer), gua_quad_coords).x;
  #if HOLE_FILLING_MODE == HOLE_FILLING_MODE_INPAINT
    if (depth == 1.0) background_color = hole_filling_inpaint();
    else              background_color = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
  #elif HOLE_FILLING_MODE == HOLE_FILLING_MODE_EPIPOLAR_SEARCH
    if (depth == 1.0) background_color = hole_filling_epipolar_search();
    else              background_color = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
  #elif HOLE_FILLING_MODE == HOLE_FILLING_MODE_EPIPOLAR_MIRROR
    if (depth == 1.0) background_color = hole_filling_epipolar_mirror();
    else              background_color = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
  #elif HOLE_FILLING_MODE == HOLE_FILLING_MODE_BLUR
    if (depth == 1.0) background_color = hole_filling_blur();
    else              background_color = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
  #else
    if (depth == 1.0) background_color = vec4(@hole_filling_color@, 1);
    else              background_color = texture2D(sampler2D(warped_color_buffer), gua_quad_coords);
  #endif

  #if WARP_MODE == WARP_MODE_RAYCASTING

    const int max_level = textureQueryLevels(usampler2D(abuf_min_max_depth));
    const vec2 total_min_max_depth = get_min_max_depth(ivec2(0), max_level);

    // gua_out_color = vec3(get_min_max_depth(ivec2(gua_quad_coords*gua_resolution)/2, 1).x);
    // return;

    int sample_count = 0;
    int abuffer_sample_count = 0;
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

    float last_depth = 0;

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

      const bool intersects = d_range.x < min_max_depth.y && min_max_depth.x < d_range.y;


      if (intersects && current_level == 0) {
        // check abuffer
        uvec2 frag = unpackUint2x32(frag_list[gua_resolution.x * int(cell_origin.y) + int(cell_origin.x)]);
        ++abuffer_sample_count;
        while (frag.x != 0) {
          float z = unpack_depth24(frag.y);
          const float thickness = 0.0005;
          if (last_depth < z-thickness && d_range.y > z && d_range.x <= z+thickness) {
            uvec4 data = frag_data[frag.x - abuf_list_offset];
            float frag_alpha = float(bitfieldExtract(frag.y, 0, 8)) / 255.0;
            vec3  frag_color = uintBitsToFloat(data.rgb);
            abuf_mix_frag(vec4(frag_color, frag_alpha), color);
            last_depth = z;

            if (color.a > @abuf_blending_termination_threshold@) {
              break;
            }
          }

          frag = unpackUint2x32(frag_list[frag.x]);
        }
      }

      #if @debug_bounding_volumes@ == 1
        // draw debug hierachy
        if (current_level == 0 && intersects) {
          abuf_mix_frag(vec4(vec3(1, 0, 0), 1), color);
          // color += vec4(1, 0, 0, 0);
        }
        float intensity = 1-pow(float(current_level) / max_level, 1);
          // color += vec4(heat(1-intensity)*0.05, 0);
        abuf_mix_frag(vec4(heat(intensity), 0.02), color);
      #endif


      if (intersects) {

        if (current_level == 0) {
          pos = new_pos;
        } else {

          // move pos to cell boundary if entering from top
          if (d_range.x < min_max_depth.x) {
            pos = pos + dir.xy*(min_max_depth.x - d_range.x) / dir.z;
          }

          --current_level;
        }
      } else {
        pos = new_pos;

        if (any(equal(mod(pos / cell_size, 2), vec2(0)))) {
          current_level = min(current_level+1, max_level);
        }
      }

      if (!intersects && at_end) {
        break;
      }

    }


    #if @debug_sample_count@ == 1
      // draw debug sample count
      color = mix(color, vec4(heat(float((abuffer_sample_count-1)*perform_ray_casting) / MAX_RAY_STEPS), 1), 0.9);
      // color = mix(color, vec4(heat(float((sample_count-1)*perform_ray_casting) / MAX_RAY_STEPS), 1), 0.9);
    #endif

    abuf_mix_frag(background_color, color);
    gua_out_color = toneMap(color.rgb);

    #if @debug_sample_ray@ == 1
      draw_debug_views();
    #endif

  #else
    gua_out_color = background_color.rgb;
  #endif

  // draw epipol
  #if @debug_epipol@
    vec2 epi = get_epipolar_direction();

    if (abs(epi.x) > abs(epi.y)) {
      epi.xy = epi.yx;
    }

    if (mod(ceil(150.0*abs(epi.x/epi.y)), 10) == 1) {
      gua_out_color = vec3(1, 0, 0);
    }

  #endif
}
