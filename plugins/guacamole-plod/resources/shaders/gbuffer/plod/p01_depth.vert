@include "resources/shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "resources/shaders/common/gua_camera_uniforms.glsl"

// input attributms
layout(location = 0) in vec3 in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 4) in float empty;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;

uniform uint gua_material_id;
uniform float radius_model_scaling;

out VertexData {
  //output to geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  //vec3 pass_ms_center;
} VertexOut;

void main() {

  vec3 ms_n = normalize(in_normal.xyz);

  vec3 ms_u;
  //compute u and v vectors
  if(ms_n.z != 0.0) {
    ms_u = vec3(1, 1, (-ms_n.x -ms_n.y)/ms_n.z);
  }
  else if(ms_n.y != 0.0) {
    ms_u = vec3(1, (-ms_n.x -ms_n.z)/ms_n.y, 1);
  }
  else {
    ms_u = vec3( (-ms_n.y -ms_n.z)/ms_n.x , 1, 1);
  }

  VertexOut.pass_ms_u = normalize(ms_u) * 1.0 * in_radius;
  VertexOut.pass_ms_v = normalize( cross(ms_n,ms_u) ) * 1.0 * in_radius;

  gl_Position = vec4(in_position, 1.0);
}

