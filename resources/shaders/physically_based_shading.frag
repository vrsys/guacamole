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
/*
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

const float Pi = 3.14159265;
const float INV_PI = 1.0f / Pi;

// uniforms
@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/brdf.glsl"

uniform mat4 gua_light_shadow_map_projection_view_matrix_0;
uniform mat4 gua_light_shadow_map_projection_view_matrix_1;
uniform mat4 gua_light_shadow_map_projection_view_matrix_2;
uniform mat4 gua_light_shadow_map_projection_view_matrix_3;

uniform uvec2 gua_shadow_map;
uniform vec3  gua_light_color;
uniform float gua_light_brightness;
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

vec3  gua_light_radiance = vec3(0.0);

// methods ---------------------------------------------------------------------
@include "shaders/common/gua_gbuffer_input.glsl"

// -----------------------------------------------------------------------------
// shadow calculations ---------------------------------------------------------
// -----------------------------------------------------------------------------

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

float gua_get_shadow(vec3 position, mat4 shadow_map_coords_matrix, vec2 lookup_offset, float acne_offset) {
  if(!gua_light_casts_shadow) {
    return 1.0;
  }

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

// -----------------------------------------------------------------------------
// lighting computation --------------------------------------------------------
// -----------------------------------------------------------------------------

subroutine void CalculateLightType(vec3 normal, vec3 position);
subroutine uniform CalculateLightType compute_light;

// base lighting calculations for point lights ---------------------------------
subroutine(CalculateLightType)
void gua_calculate_point_light(vec3 normal, vec3 position) {
  vec3 light_position = gua_lightinfo1;
  float light_radius  = gua_lightinfo3;

  gua_light_direction = light_position - position;
  gua_light_distance  = length(gua_light_direction);
  gua_light_direction /= gua_light_distance;

  float x = clamp(1.0 - pow( (gua_light_distance / light_radius) , 4), 0, 1);
  float falloff = x*x/ (gua_light_distance*gua_light_distance + 1);

  vec3 Cl = falloff * gua_light_color * gua_light_brightness;
  //gua_light_radiance = gua_light_color;
  gua_light_radiance = Cl;
}

// base lighting calculations for spot lights ----------------------------------
subroutine( CalculateLightType )
void gua_calculate_spot_light(vec3 normal, vec3 position) {
  vec3 light_position   = gua_lightinfo1;
  vec3 beam_direction   = gua_lightinfo2;
  float half_beam_angle = gua_lightinfo3;

  gua_light_direction = light_position - position;

  if (dot(-gua_light_direction, beam_direction) < 0) {
    discard;
  }

  gua_light_distance = length(gua_light_direction);
  gua_light_direction /= gua_light_distance;

  float beam_length = length(beam_direction);

  if (gua_light_distance > beam_length) {
    discard;
  }

  if (dot(normal, gua_light_direction) < 0) {
    discard;
  }

  float shadow = gua_get_shadow(position, gua_lightinfo4, vec2(0), gua_shadow_offset);

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

  vec3 Cl = radial_attenuation * length_attenuation * gua_light_color * gua_light_brightness;
  gua_light_radiance = Cl;
}

// base lighting calculations for point lights ---------------------------------
subroutine(CalculateLightType)
void gua_calculate_sun_light(vec3 normal, vec3 position) {
  vec3 light_direction = gua_lightinfo1;

  gua_light_direction = light_direction;
  gua_light_distance  = 0.0;

  if (dot(normal, gua_light_direction) < 0) {
    discard;
  }

  float shadow = 1.0;

  if (gua_is_inside_frustum(gua_light_shadow_map_projection_view_matrix_0, position)) {
    shadow = gua_get_shadow(position, gua_lightinfo4, vec2(0, 0), gua_shadow_offset);
  } else if (gua_is_inside_frustum(gua_light_shadow_map_projection_view_matrix_1, position)) {
    shadow = gua_get_shadow(position, gua_lightinfo5, vec2(1, 0), gua_shadow_offset*1.33);
  } else if (gua_is_inside_frustum(gua_light_shadow_map_projection_view_matrix_2, position)) {
    shadow = gua_get_shadow(position, gua_lightinfo6, vec2(0, 1), gua_shadow_offset*1.66);
  } else if (gua_is_inside_frustum(gua_light_shadow_map_projection_view_matrix_3, position)) {
    shadow = gua_get_shadow(position, gua_lightinfo7, vec2(1, 1), gua_shadow_offset*2);
  }

  gua_light_intensity = 1.0 * shadow;

  vec3 Cl = shadow * gua_light_color * gua_light_brightness;
  gua_light_radiance = Cl;
}

float sRGB_to_linear(float c)
{
  if(c < 0.04045)
    return (c < 0.0) ? 0.0: c * (1.0 / 12.92);
  else
    return pow((c + 0.055)/1.055, 2.4);
}

vec3 sRGB_to_linear(vec3 sRGB)
{
  return vec3( sRGB_to_linear(sRGB.r),
               sRGB_to_linear(sRGB.g),
               sRGB_to_linear(sRGB.b));
}

vec3 sRGB_to_linear_fused(vec3 c)
{
  return mix(vec3(c * (1.0 / 12.92)),
             pow((c + 0.055)/1.055, vec3(2.4)),
             greaterThanEqual(c, vec3(0.04045)));
}

// convert from sRGB to linear
vec3 sRGB_to_linear_simple(vec3 sRGB)
{
  return pow(sRGB, vec3(2.2));
}

// main ------------------------------------------------------------------------
void main() {
  vec3 N = gua_get_normal();
  vec3 P = gua_get_position();
  vec3 E = gua_camera_position;

  compute_light(N, P);

  // gua_light_direction is normalized by compute_light
  vec3 L = gua_light_direction;

  vec3 pbr = gua_get_pbr();

  float emit      = pbr.r;
  float metalness = pbr.b;
  float roughness = max(pbr.g, 0.0001f);

  vec3 albedo = sRGB_to_linear_fused(gua_get_color());
  vec3 cspec = mix(vec3(0.04), albedo, metalness);
  vec3 cdiff = mix(albedo, vec3(0.0),  metalness);

  vec3 Vn = normalize( E - P );
  vec3 H = normalize(L + Vn);
  float NdotL = clamp(dot( N, L ), 0, 1);

  vec3 Cl = gua_light_radiance * (1-emit);

  vec3 F = Fresnel(cspec, H, L);
  vec3 diffuse = lambert(cdiff);
  // specular = D*G*F / (4*nDotL*nDotV) = D * Vis * F
  // where Vis = G/(4*nDotL*nDotV)
  //vec3 D_Vis = vec3(GGX_Specular(roughness, N, H, Vn, L));
  vec3 D_Vis = vec3(D_and_Vis(roughness, N, H, Vn, L));
  vec3 brdf = mix(diffuse, D_Vis, F);
  vec3 col = Cl * brdf * NdotL;

  gua_out_color = col;
}

*/