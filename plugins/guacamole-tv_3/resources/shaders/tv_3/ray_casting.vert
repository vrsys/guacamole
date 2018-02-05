@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

layout(location = 0) in vec3 in_position;

out Data {
  vec3 pos_ms;
} VertexOut;

void main() {
  VertexOut.pos_ms = in_position;

  gl_Position = gua_model_view_projection_matrix *  vec4(in_position.xyz, 1.0);
}

