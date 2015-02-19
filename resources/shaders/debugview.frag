@include "common/header.glsl"

// varyings
in vec2 gua_quad_coords;

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"

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

// output
layout(location=0) out vec3 gua_out_color;

void main() {

  ivec2 fragment_position = ivec2(gl_FragCoord.xy);
  const int number_of_gbuffers = 5;

  int debug_window_width  = gua_resolution.x / number_of_gbuffers;
  int debug_window_height = (debug_view_width * gua_resolution.y) / gua_resolution.x;

  if ( fragment_position.y / debug_window_height == 0)
  {
    gua_out_color = vec3(1.0);
  }

#if 0
  ivec2 frag_pos = ivec2(gl_FragCoord.xy);

  // init light bitset
  int bitset_words = ((gua_lights_num - 1) >> 5) + 1;
  ivec2 tile = frag_pos >> @light_table_tile_power@;

  for (int sl = 0; sl < bitset_words; ++sl) {
    bitset[sl] = texelFetch(usampler3D(gua_light_bitset), ivec3(tile, sl), 0).r;
  }
#endif

  
  

#if 0
  vec4 final_color = vec4(0);
  vec3 bg_color = vec3(0);

  float depth = gua_get_depth();

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
#endif

}

