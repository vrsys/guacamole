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

// output
layout(location=0) out vec3 gua_out_color;

#if WARP_MODE == WARP_MODE_RAYCASTING

  @include "common/gua_abuffer.glsl"

  uniform float gua_tone_mapping_exposure = 1.0;

  @include "common/gua_tone_mapping.glsl"


  in vec2 gua_quad_coords;

  uniform mat4 warp_matrix;
  uniform uvec2 warped_depth_buffer;
  uniform uvec2 orig_depth_buffer;
  uniform uvec2 warped_color_buffer;

  void get_ray(vec2 screen_space_pos, inout vec3 start, inout vec3 end) {
    vec2 coords = screen_space_pos * 2 - 1;
    float depth = texture2D(sampler2D(warped_depth_buffer), screen_space_pos).x;

    vec4 s = vec4(coords, -1, 1);
    vec4 e = vec4(coords, depth*2-1, 1);

    s = warp_matrix * s; 
    e = warp_matrix * e; 

    // viewport clipping

    if (s.w < 0) {
      s = mix(s, e, (0 - s.z) / (e.z - s.z));
    }

    s /= s.w;
    e /= e.w;

    // if (s.z > 1)
    //   s = mix(s, e, (1 - s.z) / (e.z - s.z));

    if (s.x > 1) {
      s = mix(s, e, (1 - s.x) / (e.x - s.x));
    }

    if (s.x < -1) {
      s = mix(s, e, (-1 - s.x) / (e.x - s.x));
    }

    if (s.y > 1) {
      s = mix(s, e, (1 - s.y) / (e.y - s.y));
    }

    if (s.y < -1) {
      s = mix(s, e, (-1 - s.y) / (e.y - s.y));
    }


    if (e.x > 1) {
      e = mix(e, s, (1 - e.x) / (s.x - e.x));
    }

    if (e.x < -1) {
      e = mix(e, s, (-1 - e.x) / (s.x - e.x));
    }

    if (e.y > 1) {
      e = mix(e, s, (1 - e.y) / (s.y - e.y));
    }

    if (e.y < -1) {
      e = mix(e, s, (-1 - e.y) / (s.y - e.y));
    }

    start = s.xyz;
    end   = e.xyz;
  }

  void abuf_mix_frag(vec4 frag_color, inout vec4 color) {
    frag_color.rgb *= frag_color.a;
    color += mix(frag_color, vec4(0.0), color.a);
  }

  void main() {

    vec2 preview_coords = gua_quad_coords*5-0.1;

    // draw line
    vec2 cur_pos = preview_coords*2-1;
    vec3 s, e;
    get_ray(vec2(0.5), s, e);
    if(abs((e.y-s.y)*cur_pos.x - (e.x-s.x)*cur_pos.y + e.x*s.y - e.y*s.x) / (length(s.xy-e.xy)+0.00001) < 0.01
       && length(e.xy-cur_pos.xy) + length(s.xy-cur_pos.xy) - 0.01 < length(s.xy-e.xy)) {

      gua_out_color = mix(vec3(1, 0, 0), vec3(0, 1, 0), length(e.xy-cur_pos.xy)/(length(s.xy-e.xy)+0.00001));
      return;
    }
    
    // draw mini version of original depth buffer
    if (preview_coords.x < 1 && preview_coords.y < 1 && preview_coords.x > 0 && preview_coords.y > 0) {
      gua_out_color = vec3(texture2D(sampler2D(orig_depth_buffer), preview_coords).x);
      return;
    }

    // draw center dot
    if (length(vec2(0.5) - gua_quad_coords) < 0.001) {
      gua_out_color = vec3(1, 0, 1);
      return;
    }


    get_ray(gua_quad_coords, s, e);

    s.xy = vec2(vec2(gua_resolution) * (0.5 * s.xy + 0.5));
    e.xy = vec2(vec2(gua_resolution) * (0.5 * e.xy + 0.5));

    vec3 direction = e.xyz - s.xyz;

    int n = max(1, min(500, max(int(abs(direction.x)), int(abs(direction.y)))));
    vec3 step_size = direction/n;

    // int n = max(1, max(int(abs(direction.x)), int(abs(direction.y))));
    // vec3 step_size = direction/n;
    // n = min(150, n);

    vec4 color = vec4(0);

    for (float i=0; i<n; ++i) {

      vec2 current = s.xy + step_size.xy * i;
      
      float depth_start = s.z + step_size.z * (i+0);
      float depth_end   = s.z + step_size.z * (i+1);

      uvec2 frag = unpackUint2x32(frag_list[gua_resolution.x * int(current.y) + int(current.x)]);

      float thickness = 0.000;

      while (frag.x != 0) {
        
        float z = unpack_depth24(frag.y)*2-1;

        if (depth_end + thickness > z && depth_start - thickness <= z && z-0.00001 < e.z) {
          uvec4 data = frag_data[frag.x - abuf_list_offset];
          float frag_alpha = float(bitfieldExtract(frag.y, 0, 8)) / 255.0;
          vec3  frag_color = vec3(data.rgb)*0.001;
          abuf_mix_frag(vec4(frag_color, frag_alpha), color);
        }

        frag = unpackUint2x32(frag_list[frag.x]);
      } 
    }
    abuf_mix_frag(texture2D(sampler2D(warped_color_buffer), gua_quad_coords), color);
    gua_out_color = toneMap(color.rgb);
  }

#else

  in vec3 color;
  in vec3 normal;

  void main() {
    gua_out_color = color;
  }

#endif