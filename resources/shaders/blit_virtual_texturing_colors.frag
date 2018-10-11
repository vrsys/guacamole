@include "common/header.glsl"

// varying input
in vec2 gua_quad_coords;
// gbuffer input
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"

@include "common/gua_fragment_shader_input.glsl"



@include "common/gua_fragment_shader_output.glsl"

layout(binding = 0) uniform sampler2D gua_uv_buffer;
layout(binding = 1) uniform sampler2D passed_vt_colors;


void main() {

  //gua_out_color = gua_get_color();

  int screen_space_vt_index  = int(round(texture(gua_uv_buffer, gua_quad_coords).w));

  if( 0 != screen_space_vt_index ) {
  	gua_out_color = texture(passed_vt_colors, gua_quad_coords).rgb;//vec3(1.0, 0.0, 0.0);
  } else {
  	discard;
  }
  //gua_out_color = gua_get_color();
  //vec2 uv_coords = gl_FragCoord.xy / gua_resolution.xy;
  //gua_out_color =  gua_get_color(uv_coords);

  //submit_fragment(gl_FragCoord.z);
}
