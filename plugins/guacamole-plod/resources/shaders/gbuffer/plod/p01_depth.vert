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

out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_normal;
} VertexOut;

void main() {

@include "common_PLOD/PLOD_calculate_tangents.glsl"

@include "common_PLOD/PLOD_assign_tangents.glsl"

  VertexOut.pass_normal = (gua_normal_matrix * vec4(in_normal, 0.0)).xyz;

  gl_Position = vec4(in_position, 1.0);

}

