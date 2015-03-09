// 4 float layout
layout (std140, binding=0) uniform cameraBlock {
  mat4  gua_view_matrix;
  mat4  gua_projection_matrix;
  mat4  gua_inverse_projection_matrix;
  mat4  gua_inverse_projection_view_matrix;
  vec4  gua_camera_position_4;
  vec4  gua_clipping_planes[64];
  uvec2 gua_resolution;
  int   gua_clipping_plane_count;
  int   gua_view_id;
  float gua_clip_near;
  float gua_clip_far;
};

vec3 gua_camera_position = gua_camera_position_4.xyz;

uniform mat4 gua_model_matrix;
uniform mat4 gua_normal_matrix;
