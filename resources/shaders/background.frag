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

// varyings
in vec2 gua_quad_coords;

@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/gua_gbuffer_input.glsl"
@include "shaders/common/gua_resolve_pass_uniforms.glsl"

// output
layout(location=0) out vec3 gua_out_color;

// skymap
float gua_my_atan2(float a, float b) {
  return 2.0 * atan(a/(sqrt(b*b + a*a) + b));
}

vec3 gua_apply_background_texture() {
  return texture2D(sampler2D(gua_background_texture), gua_quad_coords).xyz;
}


vec3 gua_apply_skymap_texture() {
  vec3 pos = gua_get_position();
  vec3 curr_cam_pos = gua_get_current_camera_position();
  vec3 view = normalize(pos - curr_cam_pos);
  const float pi = 3.14159265359;
  float x = 0.5 + 0.5*gua_my_atan2(view.x, -view.z)/pi;
  float y = 1.0 - acos(view.y)/pi;
  vec2 texcoord = vec2(x, y);
  float l = length(normalize(gua_get_position(vec2(0, 0.5)) - curr_cam_pos) - normalize(gua_get_position(vec2(1.0, 0.5)) - curr_cam_pos));
  vec2 uv = l*(gua_get_quad_coords() - 1.0)/4.0 + 0.5;
  return textureGrad(sampler2D(gua_background_texture), texcoord, dFdx(uv), dFdy(uv)).xyz;
}

vec3 gua_apply_background_color() {
  return gua_background_color;
}

vec3 gua_apply_fog(vec3 fog_color) {
  float dist       = length(gua_get_current_camera_position - gua_get_position());
  float fog_factor = clamp((dist - gua_fog_start)/(gua_fog_end - gua_fog_start), 0.0, 1.0);
  return mix(gua_get_color(), fog_color, fog_factor);
}

void main() {
  float depth = gua_get_depth();
  if (depth < 1.0) {
    if (gua_enable_fog) {
      vec3 fog_color;

      switch (gua_background_mode) {
        case 0: // color
          fog_color = gua_apply_background_color();
          break;
        case 1: // skymap texture
          fog_color = gua_apply_skymap_texture();
          break;
        default: // quad texture
          fog_color = gua_apply_background_texture();
      }

      gua_out_color = gua_apply_fog(fog_color);

    } else {
      gua_out_color = gua_get_color();
    }
  } else {

    switch (gua_background_mode) {
      case 0: // color
        gua_out_color = gua_apply_background_color();
        break;
      case 1: // skymap texture
        gua_out_color = gua_apply_skymap_texture();
        break;
      default: // quad texture
        gua_out_color = gua_apply_background_texture();
    }
  }

}

