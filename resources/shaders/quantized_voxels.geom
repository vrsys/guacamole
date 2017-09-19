@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

layout (points) in;
layout (triangle_strip, max_vertices = 14) out;

uniform float voxelsize_sent  = 0.008;

in VertexData {
  vec3  gua_varying_object_position;
  vec4  gua_varying_color_rgba;
  vec3  gua_varying_rme;
} VertexIn[1];

out VertexData {
  vec3 gua_varying_world_position;
  vec3 gua_varying_normal;
  vec4 gua_varying_color_rgba;
  vec3 gua_varying_rme;
} VertexOut;

const int ordered_line_strip_indices[14] = {3, 2, 6, 7, 4, 2, 0, 3, 1, 6,   5, 4, 1, 0};


void main() {

  vec3 cube_object_space_center = VertexIn[0].gua_varying_object_position;
  float cube_half_side_length = voxelsize_sent / 2.0;

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

  for(int v_idx = 0; v_idx < 14; ++v_idx) {
    vec4 model_view_transformed_vertex = gua_model_view_matrix * cube_vertex_object_space_positions[ordered_line_strip_indices[v_idx]];
    VertexOut.gua_varying_world_position = model_view_transformed_vertex.xyz;
    gl_Position = gua_projection_matrix * model_view_transformed_vertex;

    EmitVertex();
  }

  EndPrimitive();

}



