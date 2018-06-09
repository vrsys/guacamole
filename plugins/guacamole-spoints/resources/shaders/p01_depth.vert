@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
//layout(location=0) in vec4 gua_in_position_plus_packed_floatified_color;
layout(location=0) in uvec2 pos_14_13_13qz_col_8_8_8qz;

@include "shaders/common/gua_camera_uniforms.glsl"

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

const uvec4 shift_vector = uvec4(18, 5, 0, 24);
const uvec4 mask_vector  = uvec4(0x3FFFu, 0x1FFFu, 0x1F, 0xFFu);

void main() {
/*
  uvec3 decoded_quantized_pos = uvec3(  ((pos_14_13_13qz_col_8_8_8qz.x >> 18) & 0x3FFFu),
  										((pos_14_13_13qz_col_8_8_8qz.x >>  5) & 0x1FFFu),
  										(((pos_14_13_13qz_col_8_8_8qz.x >>  0) & 0x1Fu) << 8) | ((pos_14_13_13qz_col_8_8_8qz.y >> 24) & 0xFF)
  									 );
 
*/

/*
  uvec3 decoded_quantized_pos = uvec3(  ((pos_14_13_13qz_col_8_8_8qz.x >> 18) & 0x3FFFu),
  										((pos_14_13_13qz_col_8_8_8qz.x >>  5) & 0x1FFFu),
  										((pos_14_13_13qz_col_8_8_8qz.x >>  0) & 0x1F) | (((pos_14_13_13qz_col_8_8_8qz.y >>  24) & 0xFFu) << 5 ) 
  									 );
*/
  uvec4 masked_and_shifted_pos = (uvec4(pos_14_13_13qz_col_8_8_8qz.xxx, pos_14_13_13qz_col_8_8_8qz.y) >> shift_vector) & mask_vector;
  uvec3 decoded_quantized_pos  = uvec3(masked_and_shifted_pos.xy, masked_and_shifted_pos.z | (masked_and_shifted_pos.w << 5) );
  vec3 unquantized_pos = conservative_bb_limit_min + decoded_quantized_pos * quant_steps;

  gl_Position = kinect_mvp_matrix * vec4(unquantized_pos, 1.0);
  gl_PointSize = point_size*1.5;
}