
@include "shaders/common/header.glsl"

// varyings
in vec2 gua_quad_coords;

@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/gua_gbuffer_input.glsl"


@include "shaders/common/gua_shading.glsl"

#define ABUF_MODE readonly
@include "shaders/common/gua_abuffer.glsl"
uint bitset[4];
const int tile_power = 2;

#define ABUF_SHADE_FUNC abuf_shade

vec3 shade_for_all_lights(vec3 color, vec3 normal, vec3 position, vec4 pbr, uint flags) {
  // pass-through check
  if ((flags & 1u) != 0)
    return color;

  float emit = pbr.r;

  vec3 frag_color = vec3(0.0);
  for (int i = 0; i < gua_lights_num; ++i) {
      if ((bitset[i>>5] & (1u << (i%32))) != 0) {
        frag_color += gua_shade(i, color /* (1.0 + emit)*/, normal, position, pbr);
      }
  }
  return frag_color;
}

vec4 abuf_shade(uint pos, float depth) {
  vec4 color  = ABUF_FRAG(pos, 0);
  vec4 pbr    = ABUF_FRAG(pos, 1);
  vec4 normal = ABUF_FRAG(pos, 2);

  vec4 screen_space_pos = vec4(gua_get_quad_coords() * 2.0 - 1.0, depth, 1.0);
  vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
  vec3 position = h.xyz / h.w;

  vec3 frag_color = shade_for_all_lights(color.rgb, normal.xyz *2.0 - 1.0, position, pbr, floatBitsToUint(pbr.w));
  return vec4(frag_color, color.a);
}


@include "shaders/common/gua_abuffer_resolve.glsl"

uniform int   background_mode;
uniform vec3  background_color;
uniform uvec2 background_texture;
uniform bool  enable_fog;
uniform float fog_start; 
uniform float fog_end;

// output
layout(location=0) out vec3 gua_out_color;

// skymap
float gua_my_atan2(float a, float b) {
  return 2.0 * atan(a/(sqrt(b*b + a*a) + b));
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

void main() {

  ivec2 frag_pos = ivec2(gl_FragCoord.xy);

  // init light bitset
  int bitset_words = ((gua_lights_num - 1) >> 5) + 1;
  ivec2 tile = frag_pos >> tile_power;
  for (int sl = 0; sl < bitset_words; ++sl)
    bitset[sl] = texelFetch(usampler3D(gua_light_bitset), ivec3(tile, sl), 0).r;


  vec4 final_color = vec4(0);
  vec3 bg_color = vec3(0);

  float depth = gua_get_depth();
  bool res = abuf_blend(final_color, depth);

  if (res) {
    vec3 background_color;
    switch (background_mode) {
      case 0: // color
        background_color = gua_apply_background_color();
        break;
      case 1: // skymap texture
        background_color = gua_apply_skymap_texture();
        break;
      default: // quad texture
        background_color = gua_apply_background_texture();
    }

    if (depth < 1) {
      if (enable_fog) {
        bg_color = gua_apply_fog(background_color);
      } else {
        //bg_color = gua_get_color();
        bg_color = shade_for_all_lights(gua_get_color(),
                                        gua_get_normal(),
                                        gua_get_position(),
                                        gua_get_pbr(),
                                        gua_get_flags());
      }
    } else {
      bg_color = background_color;
    }

    abuf_mix_frag(vec4(bg_color, 1.0), final_color);

  }

  gua_out_color = final_color.rgb;

/*
  for (int i = 0; i < gua_lights_num; ++i) {
      if ((bitset[i>>5] & (1u << (i%32))) != 0) {
        //gua_out_color += vec3(float(i)/3+0.2,0,0);// / 5.0;
        gua_out_color += gua_lights[i].color.rgb/2.0;
      }
  }//*/
}

