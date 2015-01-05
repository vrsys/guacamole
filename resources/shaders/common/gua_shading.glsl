// lights
uniform uvec2 gua_light_bitset;
uniform int gua_lights_num;

struct LightSource {
  vec4   position_and_radius; // xyz - position (or direction for sun light), w - radius
  vec4   beam_direction_and_half_angle; //  xyz - direction, w - half angle
  vec4   color;
  float  falloff;
  float  brightness;
  float  softness;
  uint   type;           // 0 - point, 1 - spot, 2 - sun
  bool   diffuse_enable;
  bool   specular_enable;
  bool   casts_shadow;
  //uint   pad;
};

layout(std140, binding=1) uniform lightBlock {
  LightSource gua_lights[4*32];
};

const float Pi = 3.14159265;
const float INV_PI = 1.0f / Pi;

@include "shaders/brdf.glsl"

// light functions
bool gua_calculate_light(int light_id,
                         vec3 normal, 
                         vec3 position, 
                         out vec3 gua_light_direction,
                         out vec3 gua_light_radiance) {
  LightSource L = gua_lights[light_id];

  float gua_light_intensity = 0.0;

  // sun light
  if (L.type == 2) {
    gua_light_direction = L.position_and_radius.xyz;
    if (dot(normal, gua_light_direction) < 0) {
      return false;
    }
    gua_light_intensity = 1.0 /* shadow*/;
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
    gua_light_intensity = 0.0;
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
    /*float shadow = gua_get_shadow(position, gua_lightinfo4, vec2(0), gua_shadow_offset);
    if(shadow <= 0.0) {
      return;
    }*/
    float to_light_angle = dot(-gua_light_direction, beam_direction/beam_length);
    float radial_attenuation = (to_light_angle - 1.0) / (L.beam_direction_and_half_angle.w - 1.0);
    if (radial_attenuation >= 1.0)
      return false;

    float length_attenuation = pow(1.0 - gua_light_distance/beam_length, L.falloff);
    radial_attenuation = pow(1.0 - radial_attenuation, L.softness);
    gua_light_intensity = radial_attenuation * length_attenuation /* shadow*/;
    vec3 Cl = radial_attenuation * length_attenuation * L.color.rgb * L.brightness;
    gua_light_radiance = Cl;
  }
  return true;
}

// convert from sRGB to linear
vec3 sRGB_to_linear_simple(vec3 sRGB)
{
  return pow(sRGB, vec3(2.2));
}

vec3 gua_shade(int light_id, vec3 color, vec3 normal, vec3 position, vec4 pbr) {

  vec3 N = normal;
  vec3 P = position;
  vec3 E = gua_camera_position;
  vec3 L, R;

  // lighting
  bool shaded = gua_calculate_light(light_id,
                                    N, P, L, R);
  if (!shaded) {
    return vec3(0.0);
  }

  float emit      = pbr.r;
  float metalness = pbr.b;
  float roughness = max(pbr.g, 0.0001);

  vec3 albedo = sRGB_to_linear_simple(color);
  vec3 cspec = 0.04 * (1.0 - metalness) + metalness * albedo;
  vec3 cdiff = albedo * (1.0 - metalness);

  vec3 Vn = normalize(E - P);
  vec3 H = normalize(L + Vn);
  float NdotL = clamp(dot(N, L), 0.0, 1.0);

  vec3 Cl = R /* (1.0-emit)*/;

  vec3 f = Fresnel(cspec, H, L);
  vec3 brdf = mix(lambert(cdiff),
                  vec3(GGX_Specular(roughness, N, H, Vn, L)),
                  f);
  vec3 col = Cl * brdf * NdotL;

  return col;
}


