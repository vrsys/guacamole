@include "shaders/common/header.glsl"

// input
layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec2 gua_in_texcoord;
layout(location=2) in vec3 gua_in_normal;

// uniforms
@include "shaders/common/gua_camera_uniforms.glsl"

void main() {
  vec3 position = (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
  gl_Position = gua_projection_matrix * gua_view_matrix * vec4(position, 1.0);
}
