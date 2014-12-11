
@include "shaders/common/header.glsl"

// varyings
in vec2 gua_quad_coords;

@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/gua_gbuffer_input.glsl"

#define ABUF_MODE readonly
@include "shaders/common/gua_abuffer.glsl"
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

  vec4 final_color = vec4(0);
  vec3 bg_color = vec3(0);

  float depth = gua_get_depth();
  bool res = abuf_blend(final_color, depth);

  if (res) {
    if (depth < 1) {
      if (enable_fog) {
        vec3 fog_color;

        switch (background_mode) {
          case 0: // color
            fog_color = gua_apply_background_color();
            break;
          case 1: // skymap texture
            fog_color = gua_apply_skymap_texture();
            break;
          default: // quad texture
            fog_color = gua_apply_background_texture();
        }

        bg_color = gua_apply_fog(fog_color);

      } else {
        bg_color = gua_get_color();
      }
    } else {

      switch (background_mode) {
        case 0: // color
          bg_color = gua_apply_background_color();
          break;
        case 1: // skymap texture
          bg_color = gua_apply_skymap_texture();
          break;
        default: // quad texture
          bg_color = gua_apply_background_texture();
      }
    }

    abuf_mix_frag(vec4(bg_color, 1.0), final_color);

  }

  gua_out_color = final_color.rgb;
}

