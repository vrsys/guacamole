@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
layout(location=0) in uvec2 pos_14_13_13qz_col_8_8_8qz;

@include "shaders/common/gua_camera_uniforms.glsl"


@include "shaders/common/gua_vertex_shader_output.glsl"
@include "shaders/common/gua_global_variable_declaration.glsl"

out vec3 pass_color;

//out vec3 pass_point_color;

//uniform mat4 kinect_model_matrix;
uniform mat4 kinect_mv_matrix;
uniform mat4 kinect_mvp_matrix;

const vec3 conservative_bb_limit_min = vec3(-1.5, -0.5, -1.5);
const vec3 conservative_bb_limit_max = vec3( 1.5,  2.5,  1.5);
const vec3 quant_steps               = vec3( (conservative_bb_limit_max.x - conservative_bb_limit_min.x) / (1<<14),
                                             (conservative_bb_limit_max.y - conservative_bb_limit_min.y) / (1<<13),
                                             (conservative_bb_limit_max.z - conservative_bb_limit_min.z) / (1<<13));


const uvec4 shift_vector = uvec4(18, 5, 0, 24);
const uvec4 mask_vector  = uvec4(0x3FFFu, 0x1FFFu, 0x1F, 0xFFu);

const uvec3 color_mask_vector  = uvec3(0xFF0000, 0xFF00, 0XFF);
const uvec3 color_shift_vector = uvec3(16, 8, 0);

//const uvec3 normal_shift_vector = uvec3(16, 1, 0);
//const uvec3 normal_mask_vector  = uvec3(0xFFFF, 0x7FFF, 0x1);
///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////


void main() {
  uvec4 masked_and_shifted_pos = (uvec4(pos_14_13_13qz_col_8_8_8qz.xxx, pos_14_13_13qz_col_8_8_8qz.y) >> shift_vector) & mask_vector;
  uvec3 decoded_quantized_pos  = uvec3(masked_and_shifted_pos.xy, masked_and_shifted_pos.z | (masked_and_shifted_pos.w << 5) );
  vec3 unquantized_pos = conservative_bb_limit_min + decoded_quantized_pos * quant_steps;

  vec3 decoded_color = vec3( (pos_14_13_13qz_col_8_8_8qz.yyy & color_mask_vector) >> color_shift_vector)/255.0f;

  pass_color  = decoded_color;

  gl_Position = kinect_mvp_matrix * vec4(unquantized_pos, 1.0);


}
