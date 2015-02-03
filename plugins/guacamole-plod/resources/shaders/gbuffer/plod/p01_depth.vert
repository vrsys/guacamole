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
layout(location = 4) in float empty;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;

uniform uint gua_material_id;
uniform float radius_importance_scaling;

out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;
} VertexOut;

void main() {

@include "common_PLOD/PLOD_calculate_tangents.glsl"

@include "common_PLOD/PLOD_assign_tangents.glsl"

  vec3 temp_normal = (gua_normal_matrix * vec4(in_normal, 0.0)).xyz;


  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //if(temp_normal.z > 0.0)
    gl_Position = vec4(in_position, 1.0);
  //else 
  //  gl_Position = vec4(1.0, 1.0, 1.0, 0.0) ;

}

