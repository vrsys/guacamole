@include "common/header.glsl"

// varying input
in vec2 gua_quad_coords;
// gbuffer input
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"

@include "common/gua_fragment_shader_input.glsl"



@include "common/gua_fragment_shader_output.glsl"

uniform sampler2D passed_vt_colors;

void main() {

  //gua_out_color = gua_get_color();
  gua_out_color = texture(passed_vt_colors, gua_quad_coords).rgb;//vec3(1.0, 0.0, 0.0);
  //gua_out_color = gua_get_color();
  //vec2 uv_coords = gl_FragCoord.xy / gua_resolution.xy;
  //gua_out_color =  gua_get_color(uv_coords);

  //submit_fragment(gl_FragCoord.z);
}
