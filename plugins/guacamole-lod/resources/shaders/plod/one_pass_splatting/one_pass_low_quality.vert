@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

// input attributms
layout(location = 0) in vec3 in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;

layout(location = 7) in int fem_vert_id_0;
layout(location = 8) in int fem_vert_id_1;
layout(location = 9) in int fem_vert_id_2;
layout(location = 10) in float fem_vert_w_0;
layout(location = 11) in float fem_vert_w_1;
layout(location = 12) in float fem_vert_w_2;

out VertexData {
  vec3 pass_normal;
  vec3 pass_color;
  float pass_radius;
} VertexOut;

void main() {

  VertexOut.pass_color  = vec3(in_r, in_g, in_b);
  VertexOut.pass_normal = in_normal; 
  VertexOut.pass_radius = in_radius;

  gl_Position           = vec4(in_position, 1.0);
}
