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
in vec3  gua_lightinfo1;
in vec3  gua_lightinfo2;
in float gua_lightinfo3;
in mat4  gua_lightinfo4;

// input from gbuffer
uniform uvec2 gua_depth_gbuffer_in;
@input_definition

// uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

uniform vec3  gua_light_color;
uniform float gua_light_falloff;
uniform float gua_light_softness;
uniform float gua_shadow_offset;
uniform float gua_light_shadow_map_portion;
uniform uvec2 gua_light_shadow_map;
uniform bool  gua_light_casts_shadow;
uniform bool  gua_light_diffuse_enable;
uniform bool  gua_light_specular_enable;

// material specific uniforms
@uniform_definition

// outputs ---------------------------------------------------------------------
@output_definition

// global variables
vec3 gua_light_direction;
float gua_light_distance;
float gua_light_intensity;

// methods ---------------------------------------------------------------------

// global gua_* methods
vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"
@include "shaders/uber_shaders/common/get_depth.glsl"
@include "shaders/uber_shaders/common/get_position.glsl"
@include "shaders/uber_shaders/common/get_material_id.glsl"

// shadow calculations ---------------------------------------------------------

float gua_get_shadow(vec4 smap_coords, ivec2 offset) {
  const mat4 acne_offset = mat4(
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, -gua_shadow_offset, 1
  );

  return textureProjOffset(
    gua_get_shadow_sampler(gua_light_shadow_map),
    acne_offset * smap_coords * vec4(
      gua_light_shadow_map_portion, gua_light_shadow_map_portion, 1.0, 1.0
    ), offset
  );
}

float gua_get_shadow(vec4 smap_coords) {
  const mat4 acne_offset = mat4(
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, -gua_shadow_offset, 1
  );

  return textureProj(
    gua_get_shadow_sampler(gua_light_shadow_map),
    acne_offset * smap_coords * vec4(
      gua_light_shadow_map_portion, gua_light_shadow_map_portion, 1.0, 1.0
    )
  );
}

float gua_get_shadow() {
  if(!gua_light_casts_shadow)
    return 1.0;

  mat4 shadow_map_coords_matrix = gua_lightinfo4;
  vec3 position = gua_get_position();
  vec4 smap_coords = shadow_map_coords_matrix * vec4(position, 1.0);

  float sum = 0;
  int x, y;

  for (y = -1; y <= 1; ++y)
    for (x = -1; x <= 1; ++x)
      sum += gua_get_shadow(smap_coords, ivec2(x, y));

  float shadow = sum / 9.0;

  return shadow;
}

// base lighting calculations for point lights ---------------------------------
subroutine void CalculateLightType();
subroutine uniform CalculateLightType compute_light;

subroutine(CalculateLightType)
void gua_calculate_point_light() {
  vec3 light_position = gua_lightinfo1;
  float light_radius  = gua_lightinfo3;
  vec3 gbuffer_normal = texture2D(gua_get_float_sampler(gua_float_gbuffer_in_1[0]),
                        gua_get_quad_coords()).xyz;

  gua_light_direction = light_position - gua_get_position();
  gua_light_distance  = length(gua_light_direction);
  gua_light_direction /= gua_light_distance;

  if (gua_light_distance > light_radius)
    discard;

  if (dot(gbuffer_normal, gua_light_direction) < 0)
    discard;

  gua_light_intensity = pow(1.0 - gua_light_distance/light_radius, gua_light_falloff);
}

// base lighting calculations for spot lights ----------------------------------
subroutine( CalculateLightType )
void gua_calculate_spot_light() {
  vec3 light_position   = gua_lightinfo1;
  vec3 beam_direction   = gua_lightinfo2;
  float half_beam_angle = gua_lightinfo3;
  vec3 gbuffer_normal   = texture2D(gua_get_float_sampler(gua_float_gbuffer_in_1[0]),
                          gua_get_quad_coords()).xyz;

  gua_light_direction = light_position - gua_get_position();

  if (dot(-gua_light_direction, beam_direction) < 0)
    discard;

  gua_light_distance = length(gua_light_direction);
  gua_light_direction /= gua_light_distance;

  float beam_length = length(beam_direction);

  if (gua_light_distance > beam_length)
    discard;

  if (dot(gbuffer_normal, gua_light_direction) < 0)
    discard;

  float shadow = gua_get_shadow();

  if(shadow <= 0.0)
    discard;

  float to_light_angle = dot(-gua_light_direction, beam_direction/beam_length);
  float radial_attenuation = (to_light_angle - 1.0) / (half_beam_angle - 1.0);

  if (radial_attenuation >= 1.0)
    discard;

  float length_attenuation = pow(1.0 - gua_light_distance/beam_length, gua_light_falloff);
  radial_attenuation = pow(1.0 - radial_attenuation, gua_light_softness);

  gua_light_intensity = radial_attenuation * length_attenuation * shadow;
}

// material specific methods
@material_methods

// main ------------------------------------------------------------------------
void main() {
  compute_light();

  // big switch, one case for each material
  @material_switch
}
