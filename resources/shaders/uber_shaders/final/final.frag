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

// input -----------------------------------------------------------------------

// input from vertex shader
in vec2 gua_quad_coords;

// input from gbuffer
uniform uvec2 gua_depth_gbuffer_in;
@input_definition

// uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

uniform vec3  gua_ambient_color;
uniform int   gua_background_mode;
uniform vec3  gua_background_color;
uniform uvec2 gua_background_texture;

// material specific uniforms
@uniform_definition

// outputs ---------------------------------------------------------------------
@output_definition

// methods ---------------------------------------------------------------------

// global gua_* methods
vec2 gua_get_quad_coords() {
  return gua_quad_coords;
}

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"
@include "shaders/uber_shaders/common/get_depth.glsl"
@include "shaders/uber_shaders/common/get_position.glsl"
@include "shaders/uber_shaders/common/get_material_id.glsl"

// material specific methods
@material_methods

// skymap
float gua_my_atan2(float a, float b) {
  return 2.0 * atan(a/(sqrt(b*b + a*a) + b));
}

void gua_apply_background_texture() {
  gua_float_gbuffer_out_0.rgb = texture2D(
    gua_get_float_sampler(gua_background_texture), gua_quad_coords).xyz;
}

void gua_apply_skymap_texture() {
  vec3 pos = gua_get_position();
  vec3 view = normalize(pos - gua_camera_position);

  const float pi = 3.14159265359;
  float x = 0.5 + 0.5*gua_my_atan2(view.x, -view.z)/pi;
  float y = 1.0 - acos(view.y)/pi;

  vec2 texcoord = vec2(x, y);

  float l = length(normalize(gua_get_position(vec2(0, 0.5)) - gua_camera_position) - 
                   normalize(gua_get_position(vec2(1, 0.5)) - gua_camera_position));

  vec2 uv = l*(gua_get_quad_coords() - 1.0)/4.0 + 0.5;

  gua_float_gbuffer_out_0.rgb = textureGrad(
    gua_get_float_sampler(gua_background_texture), 
                          texcoord, 
                          dFdx(uv), dFdy(uv)).xyz;
}

void gua_apply_background_color() {
  gua_float_gbuffer_out_0.rgb = gua_background_color;
}

// main ------------------------------------------------------------------------
void main() {

  if (gua_get_material_id() == 0) {
    switch (gua_background_mode) {
      case 0: // color
        gua_apply_background_color();
        break;
      case 1: // skymap texture
        gua_apply_skymap_texture();
        break;
      default: // quad texture
        gua_apply_background_texture();
    }
  } else {

    // big switch, one case for each material
    @material_switch

  }

}
