@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

@include "../common/vertex_layout.glsl"

out VertexData {
  vec3 pass_normal;
  vec3 pass_point_color;
  float pass_radius;
} VertexOut;

void main() {

  VertexOut.pass_point_color  = vec3(in_r, in_g, in_b);
  VertexOut.pass_normal = in_normal; 
  VertexOut.pass_radius = in_radius;

  gl_Position           = vec4(in_position, 1.0);
}
