// 4 float layout
layout (std430, binding=0) uniform cameraBlock {
  mat4  gua_view_matrix;
  mat4  gua_projection_matrix;
  mat4  gua_inverse_projection_matrix;
  mat4  gua_inverse_projection_view_matrix;
  vec4  gua_camera_position_4;
  float gua_clip_near;
  float gua_clip_far;
};

vec3 gua_camera_position = gua_camera_position_4.xyz;

uniform mat4 gua_model_matrix;
uniform mat4 gua_normal_matrix;
