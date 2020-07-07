// 4 float layout
layout (std140, binding=0) uniform cameraBlock {
  mat4  gua_view_matrix;
  mat4  gua_projection_matrix;
  mat4  gua_view_projection_matrix;
  mat4  gua_inverse_projection_matrix;
  mat4  gua_inverse_projection_view_matrix;
  vec4  gua_camera_position_4;

#if @compiled_with_multi_view_rendering@
   mat4  gua_secondary_view_matrix;
   mat4  gua_secondary_projection_matrix;
   mat4  gua_secondary_view_projection_matrix;
   mat4  gua_secondary_inverse_projection_matrix;
   mat4  gua_secondary_inverse_projection_view_matrix;
   vec4  gua_secondary_camera_position_4;
#endif


  vec4  gua_clipping_planes[64];
  uvec2 gua_resolution;
  uvec2 gua_noise_texture;
  vec4  gua_cyclops_position_4;
  int   gua_clipping_plane_count;
  int   gua_view_id;
  float gua_clip_near;
  float gua_clip_far;
};

vec3 gua_camera_position = gua_camera_position_4.xyz;
#if @get_enable_multi_view_rendering@
vec3 gua_secondary_camera_position = gua_secondary_camera_position_4.xyz;
#endif
vec3 gua_cyclops_position = gua_cyclops_position_4.xyz;

uniform mat4 gua_model_matrix;
uniform mat4 gua_model_view_matrix;
uniform mat4 gua_model_view_inverse_matrix;
uniform mat4 gua_model_view_projection_matrix;
uniform mat4 gua_normal_matrix;

#if @get_enable_multi_view_rendering@
uniform mat4 gua_secondary_model_view_matrix;
uniform mat4 gua_secondary_model_view_inverse_matrix;
uniform mat4 gua_secondary_model_view_projection_matrix;
#endif

uniform int  gua_rendering_mode; // 0: normal, 1: lowfi shadows, 2: hifi shadows


vec3 gua_get_current_camera_position() {

  #if @get_enable_multi_view_rendering@
  if(0 == gl_Layer) {
  #endif
    return gua_camera_position;
  #if @get_enable_multi_view_rendering@
  } else {
    return gua_secondary_camera_position;
  }
  #endif 
}
