const float Pi = 3.14159265;
const float INV_PI = 1.0f / Pi;

// lights

uniform uvec2 gua_light_bitset;
#if @get_enable_multi_view_rendering@
uniform uvec2 gua_secondary_light_bitset;
#endif

@include "gua_light_uniforms.glsl"
@include "../brdf.glsl"

// -----------------------------------------------------------------------------
// shadow calculations ---------------------------------------------------------
// -----------------------------------------------------------------------------

float gua_get_shadow(int light_id, vec4 smap_coords, ivec2 offset) {
  // ivec2 random_offset = offset + ivec2(ceil((texture(sampler2D(gua_noise_texture), gl_FragCoord.xy/64.0).xy * 2.0 - 1.0) * 3));
  return textureProjOffset(
    sampler2DShadow(gua_lights[light_id].shadow_map), smap_coords, offset);
}


float gua_get_shadow(int light_id, int cascade_id, vec3 position, float portion, float acne_offset, out float fade_to_next_cascade) {
  if(!(gua_lights[light_id].casts_shadow)) {
    return 1.0;
  }

  const mat4 acne = mat4(
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, -acne_offset, 1
  );

  const mat4 bias = mat4(
    0.5, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.0,
    0.5, 0.5, 0.5, 1.0
  );

  vec4 smap_coords = bias * gua_lights[light_id].projection_view_mats[cascade_id] * vec4(position, 1.0);
  smap_coords /= smap_coords.w;
  smap_coords += vec4(cascade_id, 0, 0, 0);
  smap_coords *= vec4(portion, 1.0, 1.0, 1.0);
  smap_coords  = acne * smap_coords;


  fade_to_next_cascade = 1.0;
  const float fade_range = 0.05;

  float posx = abs(0.5 - (smap_coords.x - cascade_id*portion)/portion);
  if (posx > 0.5 - fade_range) {
    fade_to_next_cascade *= 1.0 - (posx - 0.5 + fade_range)/fade_range;
  }

  float posy = abs(0.5 - smap_coords.y);
  if (posy > 0.5 - fade_range) {
    fade_to_next_cascade *= 1.0 - (posy - 0.5 + fade_range)/fade_range;
  }

  float shadow = 0.0;

  // test if 4 surrounding fragments are in shadow
  shadow += gua_get_shadow(light_id, smap_coords, ivec2(-1, -1));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2(-1,  2));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 2, -1));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 2,  2));

  if (shadow == 0.0) {
    return 0.0;
  } else if (shadow == 4.0) {
    return 1.0;
  }

  // only do expensive sampling if some fragments are in shadow and some are not
  shadow += gua_get_shadow(light_id, smap_coords, ivec2(-1,  0));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2(-1,  1));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 0, -1));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 0,  0));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 0,  1));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 0,  2));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 1, -1));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 1,  0));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 1,  1));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 1,  2));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 2,  0));
  shadow += gua_get_shadow(light_id, smap_coords, ivec2( 2,  1));

  return shadow / 16.0;
}

bool gua_point_outside_plane(vec4 plane, vec3 point) {
  return plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3] < 0;
}

bool gua_is_inside_frustum(int light_id, int cascade, vec3 point) {
  // vec4 proj = gua_lights[light_id].projection_view_mats[cascade] * vec4(point, 1.0);
  // proj /= proj.w;
  // return proj.x < 1 && proj.y < 1 && proj.z < 1 && proj.x > -1 && proj.y > -1 && proj.z > -1;

  vec4 plane = vec4(gua_lights[light_id].projection_view_mats[cascade][0][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][0][0],
                    gua_lights[light_id].projection_view_mats[cascade][1][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][1][0],
                    gua_lights[light_id].projection_view_mats[cascade][2][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][2][0],
                    gua_lights[light_id].projection_view_mats[cascade][3][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][3][0]);
  if (gua_point_outside_plane(plane, point)) return false;

  plane = vec4(gua_lights[light_id].projection_view_mats[cascade][0][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][0][0],
                    gua_lights[light_id].projection_view_mats[cascade][1][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][1][0],
                    gua_lights[light_id].projection_view_mats[cascade][2][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][2][0],
                    gua_lights[light_id].projection_view_mats[cascade][3][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][3][0]);
  if (gua_point_outside_plane(plane, point)) return false;

  plane = vec4(gua_lights[light_id].projection_view_mats[cascade][0][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][0][1],
                    gua_lights[light_id].projection_view_mats[cascade][1][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][1][1],
                    gua_lights[light_id].projection_view_mats[cascade][2][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][2][1],
                    gua_lights[light_id].projection_view_mats[cascade][3][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][3][1]);
  if (gua_point_outside_plane(plane, point)) return false;

  plane = vec4(gua_lights[light_id].projection_view_mats[cascade][0][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][0][1],
                    gua_lights[light_id].projection_view_mats[cascade][1][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][1][1],
                    gua_lights[light_id].projection_view_mats[cascade][2][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][2][1],
                    gua_lights[light_id].projection_view_mats[cascade][3][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][3][1]);
  if (gua_point_outside_plane(plane, point)) return false;

  plane = vec4(gua_lights[light_id].projection_view_mats[cascade][0][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][0][2],
                    gua_lights[light_id].projection_view_mats[cascade][1][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][1][2],
                    gua_lights[light_id].projection_view_mats[cascade][2][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][2][2],
                    gua_lights[light_id].projection_view_mats[cascade][3][3] + 
                    gua_lights[light_id].projection_view_mats[cascade][3][2]);
  if (gua_point_outside_plane(plane, point)) return false;

  plane = vec4(gua_lights[light_id].projection_view_mats[cascade][0][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][0][2],
                    gua_lights[light_id].projection_view_mats[cascade][1][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][1][2],
                    gua_lights[light_id].projection_view_mats[cascade][2][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][2][2],
                    gua_lights[light_id].projection_view_mats[cascade][3][3] - 
                    gua_lights[light_id].projection_view_mats[cascade][3][2]);
  if (gua_point_outside_plane(plane, point)) return false;

  return true;
}


// light functions
float gua_calculate_light(int light_id,
                          vec3 normal,
                          vec3 position,
                          out vec3 gua_light_direction,
                          out vec3 gua_light_radiance) {
  LightSource L = gua_lights[light_id];

  const float fading_exponent = 5;


#if @get_enable_multi_view_rendering@
  float fading = 0;
  if(gl_ViewportIndex == 0) {
    fading = pow(clamp(length(gua_camera_position - position) / L.max_shadow_distance, 0.0, 1.0), fading_exponent);    
  } else {
    fading = pow(clamp(length(gua_secondary_camera_position - position) / L.max_shadow_distance, 0.0, 1.0), fading_exponent);
  }
#else
  fading = pow(clamp(length(gua_camera_position - position) / L.max_shadow_distance, 0.0, 1.0), fading_exponent);
#endif


  // sun light
  if (L.type == 2) {
    gua_light_direction = L.position_and_radius.xyz;
    if (dot(normal, gua_light_direction) < 0) {
      return 0.0;
    }

    float shadow = 1.0;

    if (L.casts_shadow) {
      float portion = 1.0 / (L.cascade_count * 1.0);

      if (fading < 1.0) {
        for (int cascade = 0; cascade < L.cascade_count; ++cascade) {
          if (gua_is_inside_frustum(light_id, cascade, position)) {
            float fade_to_next;
            shadow = gua_get_shadow(light_id, cascade, position, portion, L.shadow_offset, fade_to_next);

            float tmp;
            if (fade_to_next < 1.0 && cascade+1 < L.cascade_count) {
              shadow = shadow*fade_to_next + (1.0-fade_to_next)*gua_get_shadow(light_id, cascade+1, position, portion, L.shadow_offset, tmp);
            }
            break;
          }
        }
      }

      shadow = mix(shadow, 1.0, fading);

      if(shadow <= 0.0) {
        return 0.0;
      }
    }

    vec3 Cl = L.color.rgb * L.brightness * shadow;
    gua_light_radiance = Cl;

    return shadow;
  }

  // point lights
  else if (L.type == 0) {
    gua_light_direction = L.position_and_radius.xyz - position;
    float gua_light_distance = length(gua_light_direction);
    gua_light_direction /= gua_light_distance;

    float shadow = 1.0;
    float portion = 1.0 / (L.cascade_count * 1.0);

    for (int cascade = 0; cascade < L.cascade_count; ++cascade) {
      if (gua_is_inside_frustum(light_id, cascade, position)) {
        float tmp;
        shadow = gua_get_shadow(light_id, cascade, position, portion, L.shadow_offset, tmp);
        break;
      }
    }

    shadow = mix(shadow, 1.0, fading);

    if(shadow <= 0.0) {
      return 0.0;
    }

    float x = clamp(1.0 - pow( (gua_light_distance / L.position_and_radius.w) , 4), 0, 1);
    float falloff = x*x/ (gua_light_distance*gua_light_distance + 1);
    vec3 Cl = falloff * L.color.rgb * L.brightness * shadow;
    gua_light_radiance = Cl;
    return shadow;
  }
  // spot lights
  else if (L.type == 1) {
    gua_light_direction = L.position_and_radius.xyz - position;
    vec3 beam_direction = L.beam_direction_and_half_angle.xyz;
    if (dot(-gua_light_direction, beam_direction) < 0) {
      return 0.0;
    }
    float gua_light_distance = length(gua_light_direction);
    gua_light_direction /= gua_light_distance;
    float beam_length = length(beam_direction);
    if (   gua_light_distance > beam_length
        || dot(normal, gua_light_direction) < 0) {
      return 0.0;
    }

    float tmp;
    float shadow = gua_get_shadow(light_id, 0, position, 1.0, L.shadow_offset, tmp);

    shadow = mix(shadow, 1.0, fading);

    if(shadow <= 0.0) {
      return 0.0;
    }
    float to_light_angle = dot(-gua_light_direction, beam_direction/beam_length);
    float radial_attenuation = (to_light_angle - 1.0) / (L.beam_direction_and_half_angle.w - 1.0);
    if (radial_attenuation >= 1.0) {
      return 0.0;
    }

    float length_attenuation = pow(1.0 - gua_light_distance/beam_length, L.falloff);
    radial_attenuation = pow(1.0 - radial_attenuation, L.softness);
    vec3 Cl = radial_attenuation * length_attenuation * L.color.rgb * L.brightness * shadow;
    gua_light_radiance = Cl;

    return shadow;
  }
  return 0.0;
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

#if @get_enable_multi_view_rendering@
  if(0 == gl_ViewportIndex) {
    T.E = gua_camera_position;
  } else {
    T.E = gua_secondary_camera_position;
  }
#else
  T.E = gua_camera_position;
#endif  
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
  float shaded = gua_calculate_light(light_id,
                                      T.N, T.P, L, R);
  if (shaded > 0.0) {
    vec3 H = normalize(L + T.V);
    float NdotL = clamp(dot(T.N, L), 0.0, 1.0);

    vec3 Cl = R /* (1.0-T.emit)*/;

    vec3 F = Fresnel(T.cspec, H, L);
    vec3 D_Vis = vec3(D_and_Vis(T.roughness, T.N, H, T.V, L)) * shaded;
    vec3 brdf = mix(T.diffuse * float(gua_lights[light_id].diffuse_enable), D_Vis * float(gua_lights[light_id].specular_enable), F);
    col = Cl * brdf * NdotL;
  }
  return col;
}
