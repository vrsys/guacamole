@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

layout (points) in;
layout (triangle_strip, max_vertices = 14) out;

in VertexData {
  vec3  gua_varying_object_position;
  vec4  gua_varying_color_rgba;
  float gua_varying_thickness;
  vec3  gua_varying_rme;
} VertexIn[1];

out VertexData {
  vec3 gua_varying_world_position;
  vec3 gua_varying_normal;
  vec4 gua_varying_color_rgba;
  vec3 gua_varying_rme;
  vec3 gua_start_point_ws_pos;
  vec3 gua_end_point_ws_pos;
} VertexOut;

const int ordered_line_strip_indices[14] = {3, 2, 6, 7, 4, 2, 0, 3, 1, 6,   5, 4, 1, 0};


void main() {

  vec3 cube_object_space_center = VertexIn[0].gua_varying_object_position;
  float cube_half_side_length = VertexIn[0].gua_varying_thickness;

  vec4 cube_vertex_object_space_positions[8] = {
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0, 1.0, -1.0) ), 1.0), //vertex 0
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0, 1.0, -1.0) ), 1.0), //vertex 1
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0,  1.0,  1.0) ), 1.0), //vertex 2
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0,  1.0, 1.0) ), 1.0), //vertex 3
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0, -1.0, -1.0) ), 1.0), //vertex 4
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0, -1.0, -1.0) ), 1.0), //vertex 5
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0, -1.0,  1.0) ), 1.0), //vertex 6
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0, -1.0,  1.0) ), 1.0), //vertex 7
  };

  VertexOut.gua_varying_normal         = vec3(1.0, 0.0, 0.0);
  VertexOut.gua_varying_color_rgba     = VertexIn[0].gua_varying_color_rgba;
  VertexOut.gua_varying_rme            = VertexIn[0].gua_varying_rme;

  mat4 gua_view_projection_matrix = gua_projection_matrix * gua_view_matrix;
  for(int v_idx = 0; v_idx < 14; ++v_idx) {
    vec4 model_transformed_vertex = gua_model_matrix * cube_vertex_object_space_positions[ordered_line_strip_indices[v_idx]];
    VertexOut.gua_varying_world_position = model_transformed_vertex.xyz;
    gl_Position = gua_view_projection_matrix * model_transformed_vertex;

    vec4 model_space_center_position = gua_model_matrix * vec4(cube_object_space_center, 1.0);
  
    VertexOut.gua_start_point_ws_pos = model_space_center_position.xyz;

    EmitVertex();
  }

  EndPrimitive();



/*
  if(enable_backface_culling == false || VertexIn[0].pass_normal.z > 0.0) {

      // --------------------------- common attributes -----------------------------------
      mat3x3 step_uv = mat3x3(gl_in[0].gl_Position.xyz,
                              VertexIn[0].pass_ms_u,
                              VertexIn[0].pass_ms_v);

      const float index_arr[8] = {-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

      // ---------------------------------------------------------------------------------
      for(int idx = 0; idx < 4; ++idx ) {
        vec3 uv_multiplier = vec3(1.0, 
                                  index_arr[idx],   
                                  index_arr[idx + 4]);

        VertexOut.pass_uv_coords        = uv_multiplier.yz;
        vec4 q_pos_ms                   = vec4( (step_uv * uv_multiplier) , 1.0);
        gl_Position                     = gua_model_view_projection_matrix * q_pos_ms;
        VertexOut.pass_world_position   = (gua_model_matrix * q_pos_ms).xyz;
        EmitVertex();
      }

      EndPrimitive();
  }
*/
}



