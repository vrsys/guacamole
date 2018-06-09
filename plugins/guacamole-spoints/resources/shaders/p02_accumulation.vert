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


const uvec4 shift_vector = uvec4(18, 5, 0, 24);
const uvec4 mask_vector  = uvec4(0x3FFFu, 0x1FFFu, 0x1F, 0xFFu);

const uvec3 color_mask_vector  = uvec3(0xFF0000, 0xFF00, 0XFF);
const uvec3 color_shift_vector = uvec3(16, 8, 0);
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
*/
  uvec4 masked_and_shifted_pos = (uvec4(pos_14_13_13qz_col_8_8_8qz.xxx, pos_14_13_13qz_col_8_8_8qz.y) >> shift_vector) & mask_vector;
  uvec3 decoded_quantized_pos  = uvec3(masked_and_shifted_pos.xy, masked_and_shifted_pos.z | (masked_and_shifted_pos.w << 5) );
  vec3 unquantized_pos = conservative_bb_limit_min + decoded_quantized_pos * quant_steps;

  vec3 decoded_color = vec3( (pos_14_13_13qz_col_8_8_8qz.yyy & color_mask_vector) >> color_shift_vector)/255.0f;

  gl_Position = kinect_mvp_matrix * vec4(unquantized_pos, 1.0);
  pass_point_color = decoded_color;
  gl_PointSize = point_size*1.5;
  //gl_Position = vec4(gua_in_position_plus_packed_floatified_color.xyz, 1.0);

}
