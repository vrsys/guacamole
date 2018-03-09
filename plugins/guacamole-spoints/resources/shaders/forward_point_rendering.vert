@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
layout(location=0) in vec4 gua_in_position_plus_packed_floatified_color;

@include "shaders/common/gua_camera_uniforms.glsl"

@material_uniforms@

@include "shaders/common/gua_vertex_shader_output.glsl"
@include "shaders/common/gua_global_variable_declaration.glsl"

@material_method_declarations_vert@


uniform mat4 kinect_model_matrix;
uniform float point_size = 1.0;
uniform float quant_step = -1.0;
///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  gua_world_position = vec3(0);
  gua_normal     = vec3(0);
  gua_tangent    = vec3(0);
  gua_bitangent  = vec3(0);
  gua_texcoords  = vec2(0.0, 0.0);
  gua_metalness  = 0.5;
  gua_roughness  = 0.5;
  gua_emissivity = 1;

  uint packed_vertex_color = floatBitsToUint(gua_in_position_plus_packed_floatified_color.a);

  float r_channel = float(((packed_vertex_color >> 24) & 0xFF)) / 255.0;
  float g_channel = float(((packed_vertex_color >> 16) & 0xFF)) / 255.0;
  float b_channel = float(((packed_vertex_color >> 8) & 0xFF)) / 255.0;

  gua_color = vec3(r_channel, g_channel, b_channel);
  //@material_method_calls_vert@

  @include "shaders/common/gua_varyings_assignment.glsl"

  vec4 in_pos = vec4(gua_in_position_plus_packed_floatified_color.xyz, 1.0);
  gl_Position = gua_projection_matrix * gua_view_matrix * kinect_model_matrix * in_pos;
  vec4 test = gua_projection_matrix * gua_view_matrix * kinect_model_matrix * vec4(quant_step,quant_step,quant_step,1.0);

  float final_point_size = point_size;
  if(quant_step > 0) {
    float half_step = quant_step / 2.0;
    vec4 left_pos = in_pos;
    left_pos.x -= half_step;
    vec4 right_pos = in_pos;
    right_pos.x += half_step;
    right_pos = gua_projection_matrix * gua_view_matrix * kinect_model_matrix * right_pos;
    left_pos = gua_projection_matrix * gua_view_matrix * kinect_model_matrix * left_pos;
    final_point_size = 30.0/length(test);
  }
  gl_PointSize = final_point_size;
  
  //gl_Position = vec4(gua_in_position_plus_packed_floatified_color.xyz, 1.0);

}
