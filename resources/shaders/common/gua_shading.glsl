// lights
uniform uvec2 gua_light_bitset;
uniform int gua_lights_num;
uniform int gua_sun_lights_num;

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
  LightSource gua_lights[@max_lights_num@];
};

const float Pi = 3.14159265;
const float INV_PI = 1.0f / Pi;

@include "../brdf.glsl"

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
  vec3 N;
  vec3 P;
  vec3 E;
  float roughness;
  vec3 cspec;
  vec3 Vn;
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

  T.Vn = normalize(T.E - T.P);
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
    vec3 H = normalize(L + T.Vn);
    float NdotL = clamp(dot(T.N, L), 0.0, 1.0);

    vec3 Cl = R /* (1.0-T.emit)*/;

    vec3 F = Fresnel(T.cspec, H, L);
    vec3 D_Vis = vec3(D_and_Vis(T.roughness, T.N, H, T.Vn, L));
    vec3 brdf = mix(T.diffuse * float(gua_lights[light_id].diffuse_enable), D_Vis * float(gua_lights[light_id].specular_enable), F);
    col = Cl * brdf * NdotL;
  }
  return col;
}
