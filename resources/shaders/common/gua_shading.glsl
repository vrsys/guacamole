const float Pi = 3.14159265;
const float INV_PI = 1.0f / Pi;

// lights
uniform uvec2 gua_light_bitset;

@include "gua_light_uniforms.glsl"
@include "../brdf.glsl"

// -----------------------------------------------------------------------------
// shadow calculations ---------------------------------------------------------
// -----------------------------------------------------------------------------

float gua_get_shadow(int light_id, vec4 smap_coords, ivec2 offset, float acne_offset) {
  const mat4 acne = mat4(
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, -acne_offset, 1
  );

  return textureProjOffset(
    sampler2DShadow(gua_lights[light_id].shadow_map), acne * smap_coords
    // * vec4(gua_light_shadow_map_portion, gua_light_shadow_map_portion, 1.0, 1.0)
    , offset
  );
}

// float gua_get_shadow(int light_id, vec4 smap_coords, float acne_offset) {
//   const mat4 acne = mat4(
//     1, 0, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 1, 0,
//     0, 0, -acne_offset, 1
//   );

//   return textureProj(
//     sampler2DShadow(gua_lights[light_id].shadow_map), acne * smap_coords
//     //* vec4(gua_light_shadow_map_portion, gua_light_shadow_map_portion, 1.0, 1.0)
//   );
// }

float gua_get_shadow(int light_id, vec3 position, vec2 lookup_offset, float acne_offset) {
  if(!(gua_lights[light_id].casts_shadow)) {
    return 1.0;
  }

  vec4 smap_coords = gua_lights[light_id].shadow_map_coords_mat * vec4(position, 1.0) + vec4(lookup_offset, 0, 0);

  float sum = 0;
  int x, y;

  for (y = -1; y <= 1; ++y) {
    for (x = -1; x <= 1; ++x) {
      sum += gua_get_shadow(light_id, smap_coords, ivec2(x, y), acne_offset);
    }
  }

  float shadow = sum / 9.0;

  return shadow;
}


// light functions
bool gua_calculate_light(int light_id,
                         vec3 normal,
                         vec3 position,
                         out vec3 gua_light_direction,
                         out vec3 gua_light_radiance) {
  LightSource L = gua_lights[light_id];

  // sun light
  if (L.type == 2) {
    gua_light_direction = L.position_and_radius.xyz;
    if (dot(normal, gua_light_direction) < 0) {
      return false;
    }
    vec3 Cl = /*shadow */ L.color.rgb * L.brightness;
    gua_light_radiance = Cl;
    return true;
  }

  gua_light_direction = L.position_and_radius.xyz - position;

  // point lights
  if (L.type == 0) {
    float gua_light_distance = length(gua_light_direction);
    gua_light_direction /= gua_light_distance;
    float x = clamp(1.0 - pow( (gua_light_distance / L.position_and_radius.w) , 4), 0, 1);
    float falloff = x*x/ (gua_light_distance*gua_light_distance + 1);
    vec3 Cl = falloff * L.color.rgb * L.brightness;
    gua_light_radiance = Cl;
  }
  // spot lights
  else if (L.type == 1) {
    vec3 beam_direction = L.beam_direction_and_half_angle.xyz;
    if (dot(-gua_light_direction, beam_direction) < 0) {
      return false;
    }
    float gua_light_distance = length(gua_light_direction);
    gua_light_direction /= gua_light_distance;
    float beam_length = length(beam_direction);
    if (   gua_light_distance > beam_length
        || dot(normal, gua_light_direction) < 0) {
      return false;
    }

    float shadow = gua_get_shadow(light_id, position,
                                  vec2(0), L.shadow_offset);
    if(shadow <= 0.0) {
      return false;
    }
    float to_light_angle = dot(-gua_light_direction, beam_direction/beam_length);
    float radial_attenuation = (to_light_angle - 1.0) / (L.beam_direction_and_half_angle.w - 1.0);
    if (radial_attenuation >= 1.0)
      return false;

    float length_attenuation = pow(1.0 - gua_light_distance/beam_length, L.falloff);
    radial_attenuation = pow(1.0 - radial_attenuation, L.softness);
    vec3 Cl = radial_attenuation * length_attenuation * L.color.rgb * L.brightness * shadow;
    gua_light_radiance = Cl;
  }
  return true;
}

vec3 sRGB_to_linear(vec3 c)
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

struct ShadingTerms
{
  // Surface position
  vec3 P;
  // Viewing direction (from surface position to eye)
  vec3 V;
  // Shading normal
  vec3 N;
  // Camera (Eye) position
  vec3 E;

  // parameters for our BSDF
  float roughness;
  vec3 cspec;
  vec3 diffuse;
};

void gua_prepare_shading(out ShadingTerms T, vec3 color, vec3 normal, vec3 position, vec3 pbr)
{
  T.N = normal;
  T.P = position;
  T.E = gua_camera_position;
  //T.emit      = pbr.r;
  float metalness = pbr.b;
  T.roughness = max(pbr.g, 0.0001);

  vec3 albedo = sRGB_to_linear(color);
  T.cspec = mix(vec3(0.04), albedo, metalness);
  vec3 cdiff = mix(albedo, vec3(0.0),  metalness);

  T.V = normalize(T.E - T.P);
  T.diffuse = lambert(cdiff);
}

vec3 gua_shade(int light_id, in ShadingTerms T)
{
  vec3 L, R;
  vec3 col = vec3(0);
  // lighting
  bool shaded = gua_calculate_light(light_id,
                                    T.N, T.P, L, R);
  if (shaded) {
    vec3 H = normalize(L + T.V);
    float NdotL = clamp(dot(T.N, L), 0.0, 1.0);

    vec3 Cl = R /* (1.0-T.emit)*/;

    vec3 F = Fresnel(T.cspec, H, L);
    vec3 D_Vis = vec3(D_and_Vis(T.roughness, T.N, H, T.V, L));
    vec3 brdf = mix(T.diffuse * float(gua_lights[light_id].diffuse_enable), D_Vis * float(gua_lights[light_id].specular_enable), F);
    col = Cl * brdf * NdotL;
  }
  return col;
}
