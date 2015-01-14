@include "resources/shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "resources/shaders/common/gua_camera_uniforms.glsl"

// input attributes
layout(location = 0) in vec3 in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 4) in float empty;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;

uniform uint gua_material_id;
uniform float height_divided_by_top_minus_bottom;
uniform float near_plane;
uniform float far_minus_near_plane;
uniform float radius_model_scaling;

//output to fragment shader
out vec3 pass_normal;
out float pass_mv_vert_depth;
out float pass_scaled_radius;

out float pass_log_depth;

void main() {

  float scaled_radius = radius_model_scaling * in_radius;
  mat4 model_view_matrix = gua_view_matrix * gua_model_matrix;
  vec4 pos_es = model_view_matrix * vec4(in_position, 1.0);

  gl_Position = gua_projection_matrix * gua_view_matrix * gua_model_matrix * vec4(in_position, 1.0);

  gl_PointSize = 2.0 * scaled_radius * (near_plane / -pos_es.z) *
                 height_divided_by_top_minus_bottom;


  pass_normal = normalize((gua_normal_matrix * vec4(in_normal, 0.0)).xyz);

  pass_mv_vert_depth = pos_es.z;
  pass_scaled_radius = scaled_radius;

  pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  gl_Position.z = - ( ( ( pos_es.z + 2*scaled_radius + (3.0 * scaled_radius) ) - near_plane) / (far_minus_near_plane * 1.0 ) );
  gl_Position.z = (gl_Position.z - 0.5) * 2.0;
  gl_Position.z *= gl_Position.w;
}

