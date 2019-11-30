@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

// input attributes
layout (location = 0) in vec3  in_position;
layout (location = 1) in float in_r;
layout (location = 2) in float in_g;
layout (location = 3) in float in_b;
layout (location = 4) in float empty;
layout (location = 5) in float in_radius;
layout (location = 6) in vec3 in_normal;

uniform uint gua_material_id;
uniform float radius_scaling;
uniform float max_surfel_radius;



out VertexData {
  //output to geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;

  vec3 pass_point_color;
  vec3 pass_normal;
} VertexOut;


void main() {
  @include "../common_LOD/PLOD_vertex_pass_through.glsl"

  VertexOut.pass_point_color = vec3(in_r, in_g, in_b);
  VertexOut.pass_normal = in_normal;

}