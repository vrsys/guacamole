@include "common/header.glsl"

// varying input
in vec2 gua_quad_coords;
// gbuffer input
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"

@include "common/gua_fragment_shader_input.glsl"

layout (location = 0) out vec3 out_vt_color;

uniform sampler2D gua_uv_buffer;


void main() {

  out_vt_color = texture(gua_uv_buffer, gua_quad_coords).rgb;
}
