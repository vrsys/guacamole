// 4 float layout
layout (std140, binding=0) uniform cameraBlock {
  mat4  gua_view_matrix;
  mat4  gua_projection_matrix;
  mat4  gua_inverse_projection_matrix;
  mat4  gua_inverse_projection_view_matrix;
  vec3  gua_camera_position;
};

uniform int   gua_eye;
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform float gua_texel_width;
uniform float gua_texel_height;
