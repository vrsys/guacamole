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


#if WARP_MODE == WARP_MODE_RAYCASTING

  @include "common/gua_abuffer.glsl"

  // output
  layout(location=0) out vec4 gua_out_color;

  in vec2 gua_quad_coords;

  uniform mat4 warp_matrix;
  uniform uvec2 depth_buffer;

  float gua_get_unscaled_depth(vec2 frag_pos) {
    return texture2D(sampler2D(depth_buffer), frag_pos).x;
  }

  void main() {

    vec2 coords = gua_quad_coords * 2 - 1;
    float depth = gua_get_unscaled_depth(gua_quad_coords)*2-1;

    vec4 s = vec4(coords, -1, 1);
    vec4 e = vec4(coords, depth, 1);

    s = warp_matrix * s;
    e = warp_matrix * e;

    s /= s.w;
    e /= e.w;


    // if (s.z > e.z) {
    //   discard;
    // }

    // viewport clipping
    // float alpha = 0;
    // if ((s.y > 1) || (s.y < -1))
    //   alpha = (s.y - ((s.y > 1) ? 1 : -1)) / (s.y - e.y);

    // if ((s.x > 1) || (s.x < -1))
    //   alpha = max(alpha, (s.x - ((s.x > 1) ? 1 : -1)) / (s.x - e.x));

    // s = mix(s, e, alpha);

    // if ((e.y > 1) || (e.y < -1))
    //   alpha = (e.y - ((e.y > 1) ? 1 : -1)) / (e.y - s.y);

    // if ((e.x > 1) || (e.x < -1))
    //   alpha = max(alpha, (e.x - ((e.x > 1) ? 1 : -1)) / (e.x - s.x));

    // e = mix(e, s, alpha);
    

    s.xy = vec2(vec2(gua_resolution) * (0.5 * s.xy + 0.5));
    e.xy = vec2(vec2(gua_resolution) * (0.5 * e.xy + 0.5));


    vec3 direction = e.xyz - s.xyz;

    int n = max(1, min(50, max(int(direction.x), int(direction.y))));
    
    vec3 step_size = direction/n;

    int transparent_pixels = 0;


    // gua_out_color = vec4((direction.xy)*3/ vec2(gua_resolution)*10 + 0.5, 0, 0.8);

    // gua_out_color = vec4(s.xy / vec2(gua_resolution), 0, 1);
    


    for (float i=0; i<n; ++i) {

      // if (i>=n) {
      //   break;
      // }

      vec2 current = s.xy + step_size.xy * i;
      
      float depth_start = s.z + step_size.z * (i-0.5);
      float depth_end   = s.z + step_size.z * (i+1.0);

      uvec2 frag = unpackUint2x32(frag_list[gua_resolution.x * int(current.y) + int(current.x)]);
      float z = unpack_depth24(frag.y)*2-1;

      float thickness = 0.0;

      if (frag.x != 0 && depth_end + thickness > z && depth_start - thickness <= z && z-0.00001 < depth) {
        uvec4 data = frag_data[frag.x - abuf_list_offset];
        float frag_alpha = float(bitfieldExtract(frag.y, 0, 8)) / 255.0;
        gua_out_color = vec4(unpackUnorm2x16(data.x), unpackUnorm2x16(data.y).x, frag_alpha);
        ++transparent_pixels;
        break;
      } 
    }

    if (transparent_pixels == 0) {
      discard;
    }
  }

#else

  layout(location=0) out vec3 gua_out_color;

  in vec3 color;
  in vec3 normal;

  void main() {
    gua_out_color = color;
  }

#endif