@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
//layout(location=0) in vec4 gua_in_position_plus_packed_floatified_color;
layout(location=0) in uvec2 pos_14_13_13qz_col_8_8_8qz;

@include "shaders/common/gua_camera_uniforms.glsl"

@material_uniforms@

@include "shaders/common/gua_vertex_shader_output.glsl"
@include "shaders/common/gua_global_variable_declaration.glsl"

@material_method_declarations_vert@

out vec3 pass_point_color;

//uniform mat4 kinect_model_matrix;
uniform mat4 kinect_mvp_matrix;
uniform float point_size = 1.0;


const vec3 conservative_bb_limit_min = vec3(-1.5, -0.5, -1.5);
const vec3 conservative_bb_limit_max = vec3( 1.5,  2.5,  1.5);
const vec3 quant_steps               = vec3( (conservative_bb_limit_max.x - conservative_bb_limit_min.x) / (1<<14),
                                             (conservative_bb_limit_max.y - conservative_bb_limit_min.y) / (1<<13),
                                             (conservative_bb_limit_max.z - conservative_bb_limit_min.z) / (1<<13));

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {
/*
  gua_world_position = vec3(0);
  gua_normal     = vec3(0);
  gua_tangent    = vec3(0);
  gua_bitangent  = vec3(0);
  gua_texcoords  = vec2(0.0, 0.0);
  gua_metalness  = 0.5;
  gua_roughness  = 0.5;
  gua_emissivity = 1;
*//*
  uint packed_vertex_color = floatBitsToUint(gua_in_position_plus_packed_floatified_color.a);

  float r_channel = float(((packed_vertex_color >> 24) & 0xFF)) / 255.0;
  float g_channel = float(((packed_vertex_color >> 16) & 0xFF)) / 255.0;
  float b_channel = float(((packed_vertex_color >> 8) & 0xFF)) / 255.0;

  vec3 separated_color = vec3(r_channel, g_channel, b_channel);


  pass_point_color = vec3(r_channel, g_channel, b_channel);




  gl_Position = kinect_mvp_matrix * vec4(gua_in_position_plus_packed_floatified_color.xyz, 1.0);
*/

  uvec3 decoded_quantized_pos = uvec3(  ((pos_14_13_13qz_col_8_8_8qz.x >> 18) & 0x3FFFu),
  										((pos_14_13_13qz_col_8_8_8qz.x >>  5) & 0x1FFFu),
  										(((pos_14_13_13qz_col_8_8_8qz.x >>  0) & 0x1Fu) << 8) | ((pos_14_13_13qz_col_8_8_8qz.y >> 24) & 0xFF)
  									 );
  vec3 unquantized_pos = conservative_bb_limit_min + decoded_quantized_pos * quant_steps;

  vec3 decoded_color = vec3( ((pos_14_13_13qz_col_8_8_8qz.y & 0xFF0000) >> 16)/255.0f,
  							 ((pos_14_13_13qz_col_8_8_8qz.y &   0xFF00) >>  8)/255.0f,
  							 ((pos_14_13_13qz_col_8_8_8qz.y &     0xFF) >>  0)/255.0f );

  gl_Position = kinect_mvp_matrix * vec4(unquantized_pos, 1.0);
  pass_point_color = decoded_color;
  gl_PointSize = point_size / 2.0;
  //gl_Position = vec4(gua_in_position_plus_packed_floatified_color.xyz, 1.0);

}
