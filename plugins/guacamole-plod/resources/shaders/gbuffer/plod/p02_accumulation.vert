@include "resources/shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "resources/shaders/common/gua_camera_uniforms.glsl"

// input attributes
layout (location = 0) in vec3  in_position;
layout (location = 1) in float in_r;
layout (location = 2) in float in_g;
layout (location = 3) in float in_b;
layout (location = 4) in float empty;
layout (location = 5) in float in_radius;
layout (location = 6) in vec3 in_normal;

uniform uint gua_material_id;
uniform float height_divided_by_top_minus_bottom;
uniform float near_plane;
uniform float far_minus_near_plane;
uniform float radius_model_scaling;

uniform mat4 transposed_inverse_model_matrix;

//output to fragment shader
out vec3 pass_point_color;
out vec3 pass_normal;
out vec3 pass_transposed_inverse_normal;
out float pass_mv_vert_depth;
out float pass_scaled_radius;
out float pass_screen_space_splat_size;
out float pass_view_scaling;

void main() {

  float scaled_radius = radius_model_scaling * in_radius;
  mat4 model_view_matrix = gua_view_matrix * gua_model_matrix;
  vec4 pos_es = model_view_matrix * vec4(in_position, 1.0);
/*
  vec4 pos_ss_up = gua_projection_matrix * model_view_matrix *
                   vec4(in_position + vec3(0.0, in_radius, 0.0), 1.0);
  vec4 pos_ss_down = gua_projection_matrix * model_view_matrix *
                   vec4(in_position + vec3(0.0, -in_radius, 0.0), 1.0);
*/


  //gl_Position = gua_projection_matrix * pos_es;
  //TODO: temp fix
  gl_Position = gua_projection_matrix * gua_view_matrix * gua_model_matrix * vec4(in_position, 1.0);

/*
  pos_ss_up /= pos_ss_up.w;
  pos_ss_down /= pos_ss_down.w;
*/
  vec4 pos_ss_origin = gl_Position / gl_Position.w;
/*
  float distance_top_center = length(pos_ss_up-pos_ss_origin);
  float distance_bottom_center = length(pos_ss_down-pos_ss_origin);
*/
  float splat_size = 2.0 * scaled_radius * (near_plane/-pos_es.z) *
                     height_divided_by_top_minus_bottom;
  
 
    vec4 rad_vec = vec4(0.0, 2.0*in_radius, 0.0, 1.0);
    vec4 transformed_rad_vec = gua_projection_matrix * gua_view_matrix * gua_model_matrix * rad_vec;

    transformed_rad_vec;

    gl_PointSize = splat_size;
/*
  if(distance_top_center > distance_bottom_center)
    gl_PointSize = distance_top_center;
  else
    gl_PointSize = distance_bottom_center;
*/  
  //experimental section end

  //gl_PointSize = 1.0;
  pass_point_color = vec3(in_r, in_g, in_b);
  pass_normal = normalize(( gua_normal_matrix * vec4(in_normal, 0.0)).xyz);

  pass_mv_vert_depth = pos_es.z;
  pass_scaled_radius = scaled_radius;
  pass_screen_space_splat_size = splat_size ;

  gl_Position.z  =  - (  ( ( pos_es.z  + 2*scaled_radius+ ( 3.0 * scaled_radius  ) )  - near_plane) / (far_minus_near_plane * 1.0f));
                 
  gl_Position.z = (gl_Position.z - 0.5) * 2.0; 

  gl_Position.z *= gl_Position.w;
}

