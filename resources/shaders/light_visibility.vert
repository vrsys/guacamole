@include "common/header.glsl"

// input
layout(location=0) in vec3 gua_in_position;

// uniforms
@include "common/gua_camera_uniforms.glsl"

void main() {
  gl_Position = gua_model_view_projection_matrix * vec4(gua_in_position, 1.0);
}
