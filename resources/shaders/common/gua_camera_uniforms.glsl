// 4 float layout
layout (std140, binding=0) uniform cameraBlock {
  mat4  gua_view_matrix;
  mat4  gua_projection_matrix;
  mat4  gua_inverse_projection_matrix;
  mat4  gua_inverse_projection_view_matrix;
  vec4  gua_camera_position_4;
  int   gua_view_id;
  float gua_clip_near;
  float gua_clip_far;
};

vec3 gua_camera_position = gua_camera_position_4.xyz;
float gua_tone_mapping_exposure = 1.0;

uniform mat4 gua_model_matrix;
uniform mat4 gua_normal_matrix;
