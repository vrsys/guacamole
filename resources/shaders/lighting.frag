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
in mat4  gua_lightinfo5;
in mat4  gua_lightinfo6;
in mat4  gua_lightinfo7;

// input from gbuffer
uniform uvec2 gua_gbuffer_color;
uniform uvec2 gua_gbuffer_pbr;
uniform uvec2 gua_gbuffer_normal;
uniform uvec2 gua_gbuffer_depth;

uniform uvec2 gua_shadow_map;

// uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform float gua_texel_width;
uniform float gua_texel_height;

uniform mat4 gua_light_shadow_map_projection_view_matrix_0;
uniform mat4 gua_light_shadow_map_projection_view_matrix_1;
uniform mat4 gua_light_shadow_map_projection_view_matrix_2;
uniform mat4 gua_light_shadow_map_projection_view_matrix_3;

uniform vec3  gua_light_color;
uniform float gua_light_falloff;
uniform float gua_light_softness;
uniform float gua_shadow_offset;
uniform float gua_light_shadow_map_portion;
uniform uvec2 gua_light_shadow_map;
uniform bool  gua_light_casts_shadow;
uniform bool  gua_light_diffuse_enable;
uniform bool  gua_light_specular_enable;

// output
layout(location=0) out vec3 gua_out_color;

// global variables
vec3  gua_light_direction;
float gua_light_distance;
float gua_light_intensity;

// methods ---------------------------------------------------------------------

vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

@include "shaders/uber_shaders/common/get_depth.glsl"
@include "shaders/uber_shaders/common/get_position.glsl"

// shadow calculations ---------------------------------------------------------
float gua_get_shadow(vec4 smap_coords, ivec2 offset, float acne_offset) {
  const mat4 acne = mat4(
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, -acne_offset, 1
  );

  return textureProjOffset(
    sampler2DShadow(gua_shadow_map), acne * smap_coords * vec4(
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
    sampler2DShadow(gua_shadow_map), acne_offset * smap_coords * vec4(
      gua_light_shadow_map_portion, gua_light_shadow_map_portion, 1.0, 1.0
    )
  );
}

float gua_get_shadow(mat4 shadow_map_coords_matrix, vec2 lookup_offset, float acne_offset) {
  if(!gua_light_casts_shadow) {
    return 1.0;
  }

  vec3 position = gua_get_position();
  vec4 smap_coords = shadow_map_coords_matrix * vec4(position, 1.0) + vec4(lookup_offset, 0, 0);

  float sum = 0;
  int x, y;
 
  for (y = -1; y <= 1; ++y) {
    for (x = -1; x <= 1; ++x) {
      sum += gua_get_shadow(smap_coords, ivec2(x, y), acne_offset);
    }
  }

  float shadow = sum / 9.0;

  return shadow;
}

bool gua_is_inside_frustum(mat4 frustum, vec3 position) {
  vec4 proj = frustum * vec4(position, 1.0);
  proj /= proj.w;
  return (abs(proj.x) <= 1 && abs(proj.y) <= 1 && abs(proj.z) <= 1);
}

subroutine void CalculateLightType();
subroutine uniform CalculateLightType compute_light;

// base lighting calculations for point lights ---------------------------------
subroutine(CalculateLightType)
void gua_calculate_point_light() {
  vec3 light_position = gua_lightinfo1;
  float light_radius  = gua_lightinfo3;
  vec3 gbuffer_normal = texture2D(sampler2D(gua_gbuffer_normal), gua_get_quad_coords()).xyz;

  gua_light_direction = light_position - gua_get_position();
  gua_light_distance  = length(gua_light_direction);
  gua_light_direction /= gua_light_distance;

  if (gua_light_distance > light_radius) {
    discard;
  }

  if (dot(gbuffer_normal, gua_light_direction) < 0) {
    discard;
  }

  gua_light_intensity = pow(1.0 - gua_light_distance/light_radius, gua_light_falloff);
}

// base lighting calculations for spot lights ----------------------------------
subroutine( CalculateLightType )
void gua_calculate_spot_light() {
  vec3 light_position   = gua_lightinfo1;
  vec3 beam_direction   = gua_lightinfo2;
  float half_beam_angle = gua_lightinfo3;
  vec3 gbuffer_normal   = texture2D(sampler2D(gua_gbuffer_normal),   gua_get_quad_coords()).xyz;

  gua_light_direction = light_position - gua_get_position();

  if (dot(-gua_light_direction, beam_direction) < 0) {
    discard;
  }

  gua_light_distance = length(gua_light_direction);
  gua_light_direction /= gua_light_distance;

  float beam_length = length(beam_direction);

  if (gua_light_distance > beam_length) {
    discard;
  }

  if (dot(gbuffer_normal, gua_light_direction) < 0) {
    discard;
  }

  float shadow = gua_get_shadow(gua_lightinfo4, vec2(0), gua_shadow_offset);

  if(shadow <= 0.0) {
    discard;
  }

  float to_light_angle = dot(-gua_light_direction, beam_direction/beam_length);
  float radial_attenuation = (to_light_angle - 1.0) / (half_beam_angle - 1.0);

  if (radial_attenuation >= 1.0)
    discard;

  float length_attenuation = pow(1.0 - gua_light_distance/beam_length, gua_light_falloff);
  radial_attenuation = pow(1.0 - radial_attenuation, gua_light_softness);

  gua_light_intensity = radial_attenuation * length_attenuation * shadow;
}

// base lighting calculations for point lights ---------------------------------
subroutine(CalculateLightType)
void gua_calculate_sun_light() {
  vec3 light_direction = gua_lightinfo1;
  vec3 gbuffer_normal = texture2D(sampler2D(gua_gbuffer_normal), gua_get_quad_coords()).xyz;

  gua_light_direction = light_direction;
  gua_light_distance  = 0.0;

  if (dot(gbuffer_normal, gua_light_direction) < 0) {
    discard;
  }

  vec3 position = gua_get_position();

  float shadow = 1.0;

  if (gua_is_inside_frustum(gua_light_shadow_map_projection_view_matrix_0, position)) {
    shadow = gua_get_shadow(gua_lightinfo4, vec2(0, 0), gua_shadow_offset);
  } else if (gua_is_inside_frustum(gua_light_shadow_map_projection_view_matrix_1, position)) {
    shadow = gua_get_shadow(gua_lightinfo5, vec2(1, 0), gua_shadow_offset*1.33);
  } else if (gua_is_inside_frustum(gua_light_shadow_map_projection_view_matrix_2, position)) {
    shadow = gua_get_shadow(gua_lightinfo6, vec2(0, 1), gua_shadow_offset*1.66);
  } else if (gua_is_inside_frustum(gua_light_shadow_map_projection_view_matrix_3, position)) {
    shadow = gua_get_shadow(gua_lightinfo7, vec2(1, 1), gua_shadow_offset*2);
  }

  gua_light_intensity = 1.0 * shadow;
}

// main ------------------------------------------------------------------------
void main() {
  compute_light();

  gua_out_color = texture2D(sampler2D(gua_gbuffer_color), gua_get_quad_coords()).rgb;
  gua_out_color *= gua_light_intensity;
}
