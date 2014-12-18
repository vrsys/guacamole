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

// light functions
void gua_calculate_light(int light_id,
                         vec3 normal, 
                         vec3 position, 
                         out vec3  gua_light_direction,
                         out float gua_light_distance,
                         out float gua_light_intensity,
                         out vec3  gua_light_radiance) {
  LightSource L = gua_lights[light_id];

  // sun light
  if (L.type == 3) {
    gua_light_direction = L.position_and_radius.xyz;
    gua_light_distance  = 0.0;
    if (dot(normal, gua_light_direction) < 0) {
      return;
    }
    gua_light_intensity = 1.0 /* shadow*/;
    vec3 Cl = /*shadow */ L.color.rgb * L.brightness;
    gua_light_radiance = Cl;
    return;
  }

  gua_light_direction = L.position_and_radius.xyz - position;
  gua_light_distance  = length(gua_light_direction);
  gua_light_direction /= gua_light_distance;

  // point lights
  if (L.type == 0) {
    float x = clamp(1.0 - pow( (gua_light_distance / L.position_and_radius.w) , 4), 0, 1);
    float falloff = x*x/ (gua_light_distance*gua_light_distance + 1);
    vec3 Cl = falloff * L.color.rgb * L.brightness;
    gua_light_radiance = Cl;
  }
  // spot lights
  else if (L.type == 1) {
    vec3 beam_direction   = L.beam_direction_and_half_angle.xyz;
    if (dot(-gua_light_direction, beam_direction) < 0) {
      return;
    }
    float beam_length = length(beam_direction);
    if (   gua_light_distance > beam_length
        || dot(normal, gua_light_direction) < 0) {
      return;
    }
    /*float shadow = gua_get_shadow(position, gua_lightinfo4, vec2(0), gua_shadow_offset);
    if(shadow <= 0.0) {
      return;
    }*/
    float to_light_angle = dot(-gua_light_direction, beam_direction/beam_length);
    float radial_attenuation = (to_light_angle - 1.0) / (L.beam_direction_and_half_angle.w - 1.0);
    if (radial_attenuation >= 1.0)
      return;

    float length_attenuation = pow(1.0 - gua_light_distance/beam_length, L.falloff);
    radial_attenuation = pow(1.0 - radial_attenuation, L.softness);
    gua_light_intensity = radial_attenuation * length_attenuation /* shadow*/;
    vec3 Cl = radial_attenuation * length_attenuation * L.color.rgb * L.brightness;
    gua_light_radiance = Cl;
  }
}

// shading helper functions

float saturate(float x) { return clamp(x, 0.0, 1.0); }

const float Pi = 3.1459265;
const float INV_PI = 1.0f / Pi;

// // for microsoft BRDFs (microsurface normal m == h the half vector)
// // F_schlick(F0,l,h) = F0 + (1 - F0)*(1-dot(l,h))^5

// // From s2013_pbs_rad_notes.pdf
// // ===============================================================================
// // Calculates the Fresnel factor using Schlick’s approximation
// // ===============================================================================
vec3 Fresnel(vec3 specAlbedo, vec3 h, vec3 l)
{
  float lDotH = saturate(dot(l, h));
  return specAlbedo + (1.0 - specAlbedo) * pow((1.0 - lDotH), 5.0);
}
// // ===============================================================================
// // Helper for computing the GGX visibility term
// // ===============================================================================
float GGX_V1(in float m2, in float nDotX)
{
  return 1.0 / (nDotX + sqrt(m2 + (1 - m2) * nDotX * nDotX));
}

// // ===============================================================================
// // Computes the specular term using a GGX microfacet distribution, with a
// // matching geometry factor and visibility term. m is roughness, n is the surface
// // normal, h is the half vector, l is the direction to the light source, and
// // specAlbedo is the RGB specular albedo
// // ===============================================================================
float GGX_Specular(float m, vec3 n, vec3 h, vec3 v, vec3 l)
{
  float nDotL = saturate(dot(n, l));
  if(nDotL <= 0.0)
    return 0.0;
  float nDotH = saturate(dot(n, h));
  float nDotV = max(dot(n, v), 0.0001);
  float nDotH2 = nDotH * nDotH;
  float m2 = m * m;
  // Calculate the distribution term
  float d = m2 / (Pi * pow(nDotH * nDotH * (m2 - 1.0) + 1.0, 2.0));
  // Calculate the matching visibility term
  float v1i = GGX_V1(m2, nDotL);
  float v1o = GGX_V1(m2, nDotV);
  float vis = v1i * v1o;
  // Put it all together
  return d * vis;
}

vec3 gua_shade(int light_id, vec3 color, vec3 normal, vec3 position, vec4 pbr) {

  vec3 N = normal;
  vec3 P = position;
  vec3 E = gua_camera_position;

  // lighting
  vec3  gua_light_direction = vec3(0.0);
  float gua_light_distance = 0.0;
  float gua_light_intensity = 0.0;
  vec3  gua_light_radiance = vec3(0.0);
  gua_calculate_light(light_id, 
                N, 
                P, 
                gua_light_direction,
                gua_light_distance,
                gua_light_intensity,
                gua_light_radiance);

  vec3 L = gua_light_direction;

  float emit      = pbr.r;
  float metalness = pbr.b;
  float roughness = max(pbr.g, 0.0001);

  vec3 albedo = pow(color, vec3(2.2));
  vec3 cspec = 0.04 * (1.0 - metalness) + metalness * albedo;
  vec3 cdiff = albedo * (1.0 - metalness);

  vec3 Vn = normalize(E - P);
  vec3 H = normalize(L + Vn);
  float NdotL = clamp(dot(N, L), 0.0, 1.0);

  vec3 Cl = gua_light_radiance * (1.0-emit);

  vec3 f = Fresnel(cspec, H, L);
  vec3 brdf = ( 1.0 - f ) * cdiff + f*GGX_Specular(roughness, N, H, Vn, L);
  vec3 col = Cl * brdf * NdotL;

  return col;
}


