@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

// input attributms
layout(location = 0) in vec3 in_position;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;

uniform float radius_scaling;
uniform float max_surfel_size;

out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_normal;
} VertexOut;

void main() {

  @include "../common_LOD/PLOD_vertex_pass_through.glsl"

  // set surfel size to ZERO if it exceeds threshold
  if (radius_scaling * in_radius > max_surfel_size) {
    VertexOut.pass_ms_u = vec3(0.0);
    VertexOut.pass_ms_v = vec3(0.0);
  }

  VertexOut.pass_normal = in_normal;
}
