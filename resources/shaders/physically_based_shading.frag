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

// uniforms
@include "shaders/common/gua_camera_uniforms.glsl"

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

  float lightRadius = gua_lightinfo3;
  float x = clamp(1.0 - pow( (gua_light_distance / lightRadius) , 4), 0, 1);
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

float saturate(float x) { return clamp(x,0,1); }

const float Pi = 3.1459265;
const float INV_PI = 1.0f / Pi;
// for microsoft BRDFs (microsurface normal m == h the half vector)
// F_schlick(F0,l,h) = F0 + (1 - F0)*(1-dot(l,h))^5

// From s2013_pbs_rad_notes.pdf
// ===============================================================================
// Calculates the Fresnel factor using Schlick’s approximation
// ===============================================================================
vec3 Fresnel(vec3 specAlbedo, vec3 h, vec3 l)
{
  float lDotH = saturate(dot(l, h));
  return specAlbedo + (1.0f - specAlbedo) * pow((1.0f - lDotH), 5.0f);
}
// ===============================================================================
// Helper for computing the GGX visibility term
// ===============================================================================
float GGX_V1(in float m2, in float nDotX)
{
  return 1.0f / (nDotX + sqrt(m2 + (1 - m2) * nDotX * nDotX));
}

// ===============================================================================
// Computes the specular term using a GGX microfacet distribution, with a
// matching geometry factor and visibility term. m is roughness, n is the surface
// normal, h is the half vector, l is the direction to the light source, and
// specAlbedo is the RGB specular albedo
// ===============================================================================
float GGX_Specular(float m, vec3 n, vec3 h, vec3 v, vec3 l)
{
  float nDotL = saturate(dot(n, l));
  if(nDotL <= 0.0f)
    return vec3(0.0f);
  float nDotH = saturate(dot(n, h));
  float nDotV = max(dot(n, v), 0.0001f);
  float nDotH2 = nDotH * nDotH;
  float m2 = m * m;
  // Calculate the distribution term
  float d = m2 / (Pi * pow(nDotH * nDotH * (m2 - 1) + 1, 2.0f));
  // Calculate the matching visibility term
  float v1i = GGX_V1(m2, nDotL);
  float v1o = GGX_V1(m2, nDotV);
  float vis = v1i * v1o;
  // Put it all together
  return d * vis;
}

// ===============================================================================
// Helper for computing the Beckmann geometry term
//
// ===============================================================================
float Beckmann_G1(float m, float nDotX)
{
  float nDotX2 = nDotX * nDotX;
  float tanTheta = sqrt((1 - nDotX2) / nDotX2);
  float a = 1.0f / (m * tanTheta);
  float a2 = a * a;
  float g = 1.0f;
  if(a < 1.6f)
    g *= (3.535f * a + 2.181f * a2) / (1.0f + 2.276f * a + 2.577f * a2);
  return g;
}

// ===============================================================================
// Computes the specular term using a Beckmann microfacet distribution, with a
// matching geometry factor and visibility term. m is roughness, n is the surface
// normal, h is the half vector, l is the direction to the light source, and
// specAlbedo is the RGB specular albedo
// ===============================================================================
// assumes that m != 0
vec3 Beckmann_Specular(float m, vec3 n, vec3 h, vec3 v, vec3 l, vec3 specAlbedo)
{
  float nDotL = saturate(dot(n, l));
  if(nDotL <= 0.0f)
    return vec3(0.0f);
  float nDotH = saturate(dot(n, h));
  float nDotV = max(dot(n, v), 0.0001f);
  float nDotH2 = nDotH * nDotH;
  float nDotH4 = nDotH2 * nDotH2;
  float m2 = m * m;

  // Calculate the distribution term -- uses normal, halfvector h, and roughness
  // warum nicht nDotH2 - 1 ?
  float tanTheta2 = (1 - nDotH2) / nDotH2;
  float expTerm = exp(-tanTheta2 / m2);
  // warum nicht m^4 ?
  float D = expTerm / (Pi * m2 * nDotH4);
  // Calculate the matching geometric term
  float g1i = Beckmann_G1(m, nDotL);

  float g1o = Beckmann_G1(m, nDotV);
  float G = g1i * g1o;
  // Calculate the fresnel term
  //float f = Fresnel(specAlbedo, h, l);
  vec3 F = Fresnel(specAlbedo, h, l);
  // Put it all together
  return D * G * F * (1.0f / (4.0f * nDotL * nDotV));
}

// End - From s2013_pbs_rad_notes.pdf

// http://renderwonk.com/publications/s2010-shading-course/hoffman/s2010_physically_based_shading_hoffman_a_notes.pdf
// schlick , use h for n with microfacets
// f0 - reflection coefficient for light incoming parallel to the normal
vec3 F_schlick(vec3 f0, vec3 L, vec3 N)
{
  return f0 + (1.0 - f0) * pow(1.0 - saturate(dot(L, N))  , 5);
}

float F_schlick( float LdotH )
{
  //float x  = clamp( 1 - LdotH, 0.0, 1.0 );
  float x  = 1.0 - saturate(LdotH);
  float x2 = x * x;
  return ( x2 * x2 * x );
}

float F_schlick(vec3 L, vec3 N, float eta)
{
  float sqr_f0 = (1-eta)*(1+eta);
  float f0 = sqr_f0 * sqr_f0;
  return f0 + (1.0 - f0) * pow(1.0 - saturate(dot(L, N))  , 5);
}

vec3 F_schlick(float n1, float n2, float costheta)
{
  float sqr_f0 = (n1 - n2)/(n1+n2);
  float f0 = sqr_f0*sqr_f0;
  return vec3(f0 + (1.0 - f0) * pow(1.0 - saturate(costheta)  , 5));
}

vec3 fresnelSchlickWithRoughness(vec3 c_spec,vec3 E,vec3 N,float gloss)
{
  return c_spec + (max(vec3(gloss), c_spec) - c_spec) * pow(1 - saturate(dot(E, N)), 5);
}

// main ------------------------------------------------------------------------
void main() {
  vec3 N = gua_get_normal();
  vec3 P = gua_get_position();
  vec3 E = gua_camera_position;

  compute_light(N, P);

  vec3 L = gua_light_direction;

  vec3 pbr = gua_get_pbr();
  
  float emit      = pbr.r;
  float metalness = pbr.b;
  float roughness = max(pbr.g, 0.0001f);

  vec3 cspec = 0.04 * (1 - metalness) + metalness * gua_get_color();
  vec3 cdiff = gua_get_color() * (1 - metalness);

  vec3 Vn = normalize( E - P );
  vec3 H = normalize(L + Vn);
  float NdotL = clamp(dot( N, L ), 0, 1);

  vec3 Cl = gua_light_radiance * (1-emit);

  vec3 f = Fresnel(cspec, H, L);
  vec3 brdf = ( 1.0 - f ) * cdiff + f*GGX_Specular(roughness, N, H, Vn, L);
  vec3 col = Cl * brdf * NdotL;

  gua_out_color = col;
  //gua_out_color = NdotL * vec3(1.0);
}
