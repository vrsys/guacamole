@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/common/gua_camera_uniforms.glsl"

in VertexDataIn {
  vec3 ms_pos;
  vec3 color;
} VertexIn[1];

out VertexDataOut {
  vec3 ms_center_pos;
  vec3 ms_curr_pos;
  vec3 color;
  float log_depth;
} VertexOut;


//uniform mat4 kinect_model_matrix;
uniform mat4 kinect_mv_matrix;
uniform mat4 kinect_mvp_matrix;
uniform float point_size = 1.0;

uniform float voxel_half_size = 0.0;

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////


layout (points) in;
layout (triangle_strip, max_vertices = 14) out;
//layout (points = 1 /*14*/) out;

const int ordered_line_strip_indices[14] = {3, 2, 6, 7, 4, 2, 0, 3, 1, 6,   5, 4, 1, 0};


void main() {

  vec3 cube_object_space_center = VertexIn[0].ms_pos;
  float cube_half_side_length = voxel_half_size * 2.0;

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




  for(int v_idx = 0; v_idx < 14; ++v_idx) {

    vec4 model_view_transformed_vertex = kinect_mv_matrix * cube_vertex_object_space_positions[ordered_line_strip_indices[v_idx]];

    vec4 projected_vertex = gua_projection_matrix * model_view_transformed_vertex;


    //gl_Position = kinect_mvp_matrix * cube_vertex_object_space_positions[ordered_line_strip_indices[v_idx]];

    gl_Position = projected_vertex;

   // VertexOut.log_depth = (gl_Position.z - 0.5) * 2.0 / gl_Position.w;
    VertexOut.log_depth = ((gl_Position.z / gl_Position.w) / 2.0) + 0.5;


    gl_Position.z = (-(model_view_transformed_vertex.z )) / gua_clip_far;
    gl_Position.z  = (gl_Position.z - 0.5) * 2.0;
    gl_Position.z  *= gl_Position.w;


    VertexOut.ms_center_pos = cube_object_space_center;
    VertexOut.ms_curr_pos   = (cube_vertex_object_space_positions[ordered_line_strip_indices[v_idx]].xyz);

    VertexOut.color = VertexIn[0].color;
    EmitVertex();
  }

  EndPrimitive();



}



