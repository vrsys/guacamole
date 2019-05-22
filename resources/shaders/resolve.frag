@include "common/header.glsl"

// varying input
in vec2 gua_quad_coords;

// uniforms
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_resolve_pass_uniforms.glsl"

// gbuffer input
@include "common/gua_gbuffer_input.glsl"

// methods
@include "common/gua_shading.glsl"
@include "common/gua_tone_mapping.glsl"

@include "ssao.frag"
@include "screen_space_shadow.frag"

#define ABUF_MODE readonly
#define ABUF_SHADE_FUNC abuf_shade
@include "common/gua_abuffer_resolve.glsl"

uint bitset[((@max_lights_num@ - 1) >> 5) + 1];

///////////////////////////////////////////////////////////////////////////////
vec2
longitude_latitude(in vec3 normal)
{
  const float invpi = 1.0 / 3.14159265359;

  vec2 a_xz = normalize(normal.xz);
  vec2 a_yz = normalize(normal.yz);

  return vec2(0.5 * (1.0 + invpi * atan(a_xz.x, -a_xz.y)),
              acos(-normal.y) * invpi);
}

// https://www.unrealengine.com/blog/physically-based-shading-on-mobile
vec3 EnvBRDFApprox( vec3 SpecularColor, float Roughness, float NoV )
{
  const vec4 c0 = vec4(-1, -0.0275, -0.572, 0.022);
  const vec4 c1 = vec4(1, 0.0425, 1.04, -0.04);
  vec4 r = Roughness * c0 + c1;
  float a004 = min( r.x * r.x, exp2( -9.28 * NoV ) ) * r.x + r.y;
  vec2 AB = vec2( -1.04, 1.04 ) * a004 + r.zw;
  return SpecularColor * AB.x + AB.y;
}

///////////////////////////////////////////////////////////////////////////////
vec3 environment_lighting (in ShadingTerms T)
{
  vec3 env_color = vec3(0);
  vec3 brdf_spec = EnvBRDFApprox(T.cspec, T.roughness, dot(T.N, T.V));
  vec3 col1 = vec3(0);
  vec3 col2 = vec3(0);

  switch (gua_environment_lighting_mode) {
    case 0 : // spheremap
      vec2 texcoord = longitude_latitude(T.N);
      col1 = brdf_spec * texture(sampler2D(gua_environment_lighting_texture), texcoord).rgb;
      col2 = brdf_spec * texture(sampler2D(gua_alternative_environment_lighting_texture), texcoord).rgb;
      env_color = mix(col1, col2, gua_environment_lighting_texture_blend_factor);
      break;
    case 1 : // cubemap
      col1 = brdf_spec * texture(samplerCube(gua_environment_lighting_texture), T.N).rgb;
      col2 = brdf_spec * texture(samplerCube(gua_alternative_environment_lighting_texture), T.N).rgb;
      env_color = mix(col1, col2, gua_environment_lighting_texture_blend_factor);
      break;
    case 2 : // single color
      // http://marmosetco.tumblr.com/post/81245981087
      float gua_horizon_fade = 1.3;
      vec3 R = reflect(-T.V, T.N);
      float horizon = saturate( 1.0 + gua_horizon_fade * dot(R, T.N));
      horizon *= horizon;
      vec3 brdf_diff = T.diffuse;
      env_color = (Pi * brdf_diff + (horizon * brdf_spec)) * gua_horizon_fade * gua_environment_lighting_color;
      break;
  };

  return env_color;
}

///////////////////////////////////////////////////////////////////////////////
vec3 shade_for_all_lights(in vec3 color, in vec3 normal, in vec3 position, in vec3 pbr, in uint flags, in float depth, in bool ssao_enable) {

  float emit = pbr.r;

  // pass-through check
  if (emit == 1.0) {
    return color;
  }

  ShadingTerms T;
  gua_prepare_shading(T, color, normal, position, pbr);

  vec3 frag_color = vec3(0);
  for (int i = 0; i < gua_lights_num; ++i) {
      float screen_space_shadow = 0.0;

      if(gua_screen_space_shadows_enable) {
        screen_space_shadow = compute_screen_space_shadow (i, position);
      }

      // is it either a visible spot/point light or a sun light ?
      if ( ((bitset[i>>5] & (1u << (i%32))) != 0)
         || i >= gua_lights_num - gua_sun_lights_num )
      {
        frag_color += (1.0 - screen_space_shadow) * gua_shade(i, T);
      }
  }

  float ambient_occlusion = 0.0;
  if (ssao_enable) {
    ambient_occlusion = compute_ssao(normal, position, depth);
  }
  frag_color += (1.0 - ambient_occlusion) * environment_lighting(T);

  return mix(frag_color, color, emit);
}

///////////////////////////////////////////////////////////////////////////////
#if @enable_abuffer@
vec4 abuf_shade(uint pos, float depth) {

  uvec4 data = frag_data[pos];

  vec3 color = vec3(unpackUnorm2x16(data.x), unpackUnorm2x16(data.y).x);
  vec3 normal = vec3(unpackSnorm2x16(data.y).y, unpackSnorm2x16(data.z));
  vec3 pbr = unpackUnorm4x8(data.w).xyz;
  uint flags = bitfieldExtract(data.w, 24, 8);

  vec4 screen_space_pos = vec4(gua_get_quad_coords() * 2.0 - 1.0, depth, 1.0);
  vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
  vec3 position = h.xyz / h.w;

  vec4 frag_color_emit = vec4(shade_for_all_lights(color, normal, position, pbr, flags, depth, false), pbr.r);
  return frag_color_emit;
}
#endif

///////////////////////////////////////////////////////////////////////////////

// output
layout(location=0) out vec3 gua_out_color;


///////////////////////////////////////////////////////////////////////////////

// skymap

float gua_my_atan2(float a, float b) {
  return 2.0 * atan(a/(sqrt(b*b + a*a) + b));
}

///////////////////////////////////////////////////////////////////////////////
vec3 gua_apply_background_texture() {
  vec3 col1 = texture(sampler2D(gua_background_texture), gua_quad_coords).xyz;
  vec3 col2 = texture(sampler2D(gua_alternative_background_texture), gua_quad_coords).xyz;
  return mix(col1, col2, gua_background_texture_blend_factor);
}

///////////////////////////////////////////////////////////////////////////////
vec3 gua_apply_cubemap_texture() {
  vec3 pos = gua_get_position();
  vec3 view = normalize(pos - gua_camera_position) ;
  vec3 col1 = texture(samplerCube(gua_background_texture), view).xyz;
  vec3 col2 = texture(samplerCube(gua_alternative_background_texture), view).xyz;
  return mix(col1, col2, gua_background_texture_blend_factor);
}

///////////////////////////////////////////////////////////////////////////////
vec3 gua_apply_skymap_texture() {
  vec3 pos = gua_get_position();
  vec3 view = normalize(pos - gua_camera_position);
  const float pi = 3.14159265359;
  float x = 0.5 + 0.5*gua_my_atan2(view.x, -view.z)/pi;
  float y = 1.0 - acos(view.y)/pi;
  vec2 texcoord = vec2(x, y);
  float l = length(normalize(gua_get_position(vec2(0, 0.5)) - gua_camera_position) - normalize(gua_get_position(vec2(1, 0.5)) - gua_camera_position));
  vec2 uv = l*(gua_get_quad_coords() - 1.0)/4.0 + 0.5;
  vec3 col1 = textureGrad(sampler2D(gua_background_texture), texcoord, dFdx(uv), dFdy(uv)).xyz;
  vec3 col2 = textureGrad(sampler2D(gua_alternative_background_texture), texcoord, dFdx(uv), dFdy(uv)).xyz;
  return mix(col1, col2, gua_background_texture_blend_factor);
}

///////////////////////////////////////////////////////////////////////////////
vec3 gua_apply_background_color() {
  return gua_background_color;
}

///////////////////////////////////////////////////////////////////////////////
vec3 gua_apply_fog(vec3 color, vec3 fog_color) {
  float dist       = length(gua_camera_position - gua_get_position());
  float fog_factor = clamp((dist - gua_fog_start)/(gua_fog_end - gua_fog_start), 0.0, 1.0);
  return mix(color, fog_color, fog_factor);
}

///////////////////////////////////////////////////////////////////////////////
vec3 gua_get_background_color() {
  switch (gua_background_mode) {
    case 0: // color
      return sRGB_to_linear(gua_apply_background_color());
    case 1: // skymap texture
      return sRGB_to_linear(gua_apply_skymap_texture());
    case 2: // quad texture
      return sRGB_to_linear(gua_apply_background_texture());
  }
  // cubemap
  return sRGB_to_linear(gua_apply_cubemap_texture());
}

///////////////////////////////////////////////////////////////////////////////
float get_vignette(float coverage, float softness, float intensity) {
  // inigo quilez's great vigneting effect!
  float a = -coverage/softness;
  float b = 1.0/softness;
  vec2 q = gua_get_quad_coords();
  return clamp(a + b*pow( 16.0*q.x*q.y*(1.0-q.x)*(1.0-q.y), 0.1 ), 0.0, 1.0) * intensity + (1.0-intensity);
}

///////////////////////////////////////////////////////////////////////////////
void main() {
  //gua_out_color = gua_get_color() + gua_get_normal() + gua_get_pbr() + gua_get_position() + vec3(gua_get_flags());


  ivec2 frag_pos = ivec2(gl_FragCoord.xy);

  // init light bitset
  int bitset_words = ((gua_lights_num - 1) >> 5) + 1;
  ivec2 tile = frag_pos >> @light_table_tile_power@;
  for (int sl = 0; sl < bitset_words; ++sl) {
    bitset[sl] = texelFetch(usampler3D(gua_light_bitset), ivec3(tile, sl), 0).r;
  }

  vec4 abuffer_accumulation_color = vec4(0);
  float abuffer_accumulation_emissivity = 0.0;
  vec3 gbuffer_color = vec3(0);

  // unscaled depth is in [0, 1]
  float unscaled_depth = gua_get_unscaled_depth();

  // depth is in [-1, 1]
  float depth = gua_scale_unscaled_depth(unscaled_depth);

#if @enable_abuffer@
  bool res = abuf_blend(abuffer_accumulation_color, abuffer_accumulation_emissivity, unscaled_depth);
#else
  bool res = true;
#endif



    vec3 gbuf_pbr      = gua_get_pbr();

    vec3 background_color = vec3(0.0);

    bool depth_smaller_one = depth >= 1.0;
    if(gua_enable_fog || (res && depth_smaller_one) ) {
      background_color = gua_get_background_color();
    }

  if (res) {
    if (depth < 1) {

      vec3 gua_get_color = gua_get_color();
      vec3 gbuf_normal   = gua_get_normal();
      vec3 gbuf_position = gua_unproject_depth_to_position(depth);
      uint gbuf_flags    = gua_get_flags();
      
      gbuffer_color += shade_for_all_lights(gua_get_color,
                                            gbuf_normal,
                                            gbuf_position,
                                            gbuf_pbr,
                                            gbuf_flags,
                                            depth,
                                            gua_ssao_enable);
      if (gua_enable_fog) {
        gbuffer_color = gua_apply_fog(gbuffer_color, background_color);
      }
    }
    else {
      gbuffer_color += background_color;
    }

    abuffer_accumulation_emissivity += gbuf_pbr.r * (1-abuffer_accumulation_color.a);
    abuf_mix_frag(vec4(gbuffer_color, 1.0), abuffer_accumulation_color);
  }

  gua_out_color = mix(toneMap(abuffer_accumulation_color.rgb), abuffer_accumulation_color.rgb, abuffer_accumulation_emissivity);

  // vignette
  if (gua_vignette_color.a > 0) {
    float vignetting = get_vignette(gua_vignette_coverage, gua_vignette_softness, gua_vignette_color.a);
    gua_out_color = mix(gua_vignette_color.rgb, gua_out_color, vignetting);
  }

  // color correction

#if @gua_debug_tiles@
  vec3 color_codes[] = {vec3(1,0,0), vec3(0,1,0), vec3(0,0,1), vec3(1,1,0), vec3(1,0,1), vec3(0,1,1)};

  for (int i = 0; i < gua_lights_num; ++i) {

    if ((bitset[i>>5] & (1u << (i%32))) != 0) {

      gua_out_color = mix(gua_out_color, gua_lights[i].color.rgb, 0.2);
      int ts = int(pow(2, @light_table_tile_power@));

      if (@light_table_tile_power@ > 2) {

        bool p1 = any(equal(frag_pos % ts, vec2(0)));
        bool p2 = any(equal(frag_pos % ts, vec2(ts-1)));
        if (p1 || p2) gua_out_color = vec3(0);
        ivec2 tpos = (frag_pos >> @light_table_tile_power@) * ivec2(ts);

        if (  all(greaterThanEqual(frag_pos, tpos + ivec2(2+i*4, 2)))
            && all(lessThanEqual(frag_pos, tpos + ivec2(6+i*4, 6))))
          gua_out_color = color_codes[i % color_codes.length()];

      }
    }
  }
#endif


}

