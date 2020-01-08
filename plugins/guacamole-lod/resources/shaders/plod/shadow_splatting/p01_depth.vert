@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

@include "../common/vertex_layout.glsl"

uniform float radius_scaling;
uniform float max_surfel_radius;

out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_normal;
} VertexOut;

void main() {

@include "../common_LOD/PLOD_vertex_pass_through.glsl"

}

