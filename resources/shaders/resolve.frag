@include "common/header.glsl"

// varyings
in vec2 gua_quad_coords;

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"
@include "common/gua_shading.glsl"
@include "common/gua_tone_mapping.glsl"

#define ABUF_MODE readonly
#define ABUF_SHADE_FUNC abuf_shade
@include "common/gua_abuffer_resolve.glsl"

uint bitset[((@max_lights_num@ - 1) >> 5) + 1];

vec3 shade_for_all_lights(vec3 color, vec3 normal, vec3 position, vec3 pbr, uint flags) {
  // pass-through check
  if ((flags & 1u) != 0)
    return color;

  float emit = pbr.r;
  ShadingTerms T;
  gua_prepare_shading(T, color/* (1.0 + emit)*/, normal, position, pbr);

  vec3 frag_color = vec3(0.0);
  for (int i = 0; i < gua_lights_num; ++i) {
      if ((bitset[i>>5] & (1u << (i%32))) != 0) {
        frag_color += gua_shade(i, T);
      }
  }
  return toneMap(frag_color);
}

#if @enable_abuffer@
vec3 abuf_shade(uint pos, float depth) {
  vec4 color  = ABUF_FRAG(pos, 0);
  vec4 normal = ABUF_FRAG(pos, 1);
  vec4 pbr = unpackUnorm4x8(floatBitsToUint(color.w));
  uint flags = bitfieldExtract(floatBitsToUint(color.w), 24, 8);

  vec4 screen_space_pos = vec4(gua_get_quad_coords() * 2.0 - 1.0, depth, 1.0);
  vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
  vec3 position = h.xyz / h.w;

  vec3 frag_color = shade_for_all_lights(color.rgb, fma(normal.xyz, vec3(2.0), vec3(-1.0)), position, pbr.rgb, flags);
  return frag_color;
}
#endif

uniform vec3  background_color;
uniform uvec2 background_texture;
uniform bool  enable_fog;
uniform float fog_start; 
uniform float fog_end;

// output
layout(location=0) out vec3 gua_out_color;

// skymap
float gua_my_atan2(float a, float b) {
  return 2.0 * atan(fma(a, inversesqrt(b*b + a*a), b));
}

vec3 gua_apply_background_texture() {
  return texture2D(sampler2D(background_texture), gua_quad_coords).xyz;
}

vec3 gua_apply_skymap_texture() {
  vec3 pos = gua_get_position();
  vec3 view = normalize(pos - gua_camera_position);
  const float pi = 3.14159265359;
  float x = 0.5 + 0.5*gua_my_atan2(view.x, -view.z)/pi;
  float y = 1.0 - acos(view.y)/pi;
  vec2 texcoord = vec2(x, y);
  float l = length(normalize(gua_get_position(vec2(0, 0.5)) - gua_camera_position) - normalize(gua_get_position(vec2(1, 0.5)) - gua_camera_position));
  vec2 uv = l*(gua_get_quad_coords() - 1.0)/4.0 + 0.5;
  return textureGrad(sampler2D(background_texture), texcoord, dFdx(uv), dFdy(uv)).xyz;
}

vec3 gua_apply_background_color() {
  return background_color;
}

vec3 gua_apply_fog(vec3 fog_color) {
  float dist       = length(gua_camera_position - gua_get_position());
  float fog_factor = clamp((dist - fog_start)/(fog_end - fog_start), 0.0, 1.0);
  return mix(gua_get_color(), fog_color, fog_factor);
}

vec3 gua_get_background_color() {
  switch (@background_mode@) {
    case 0: // color
      return gua_apply_background_color();
    case 1: // skymap texture
      return gua_apply_skymap_texture();
  }
  // quad texture
  return gua_apply_background_texture();
}

void main() {

  ivec2 frag_pos = ivec2(gl_FragCoord.xy);

  // init light bitset
  int bitset_words = ((gua_lights_num - 1) >> 5) + 1;
  ivec2 tile = frag_pos >> @light_table_tile_power@;
  for (int sl = 0; sl < bitset_words; ++sl)
    bitset[sl] = texelFetch(usampler3D(gua_light_bitset), ivec3(tile, sl), 0).r;


  vec4 final_color = vec4(0);
  vec3 bg_color = vec3(0);

  float depth = gua_get_depth();
  //gua_out_color=vec3(depth); return;

#if @enable_abuffer@
  bool res = abuf_blend(final_color, depth);
  //gua_out_color = final_color.rgb; return;
#else
  bool res = true;
#endif

  if (res) {
    if (depth < 1) {
      if (enable_fog) {
        bg_color = gua_apply_fog(gua_get_background_color());
      }
      else {
        //bg_color = gua_get_color();
        bg_color = shade_for_all_lights(gua_get_color(),
                                        gua_get_normal(),
                                        gua_get_position(),
                                        gua_get_pbr(),
                                        gua_get_flags());
      }
    }
    else {
      bg_color = gua_get_background_color();
    }

    abuf_mix_frag(vec4(bg_color, 1.0), final_color);
  }

  gua_out_color = final_color.rgb;

#if @debug_tiles@
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

