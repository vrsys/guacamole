@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
//layout(location=0) in vec4 gua_in_position_plus_packed_floatified_color;
layout(location=0) in uvec3 pos_14_13_13qz_col_8_8_8qz;

@include "shaders/common/gua_camera_uniforms.glsl"

@material_uniforms@

@include "shaders/common/gua_vertex_shader_output.glsl"
@include "shaders/common/gua_global_variable_declaration.glsl"

@material_method_declarations_vert@

out VertexDataIn {
  vec3 ms_pos;
  vec3 color;
  vec3 ms_u;
  vec3 ms_v;
  vec3 ms_normal;
} VertexOut;

//out vec3 pass_point_color;

//uniform mat4 kinect_model_matrix;
uniform mat4 kinect_mv_matrix;
uniform mat4 kinect_mvp_matrix;
uniform float point_size = 1.0;

uniform float voxel_half_size = 0.0;
const vec3 conservative_bb_limit_min = vec3(-1.5, -0.5, -1.5);
const vec3 conservative_bb_limit_max = vec3( 1.5,  2.5,  1.5);
const vec3 quant_steps               = vec3( (conservative_bb_limit_max.x - conservative_bb_limit_min.x) / (1<<14),
                                             (conservative_bb_limit_max.y - conservative_bb_limit_min.y) / (1<<13),
                                             (conservative_bb_limit_max.z - conservative_bb_limit_min.z) / (1<<13));


const uvec4 shift_vector = uvec4(18, 5, 0, 24);
const uvec4 mask_vector  = uvec4(0x3FFFu, 0x1FFFu, 0x1F, 0xFFu);

const uvec3 color_mask_vector  = uvec3(0xFF0000, 0xFF00, 0XFF);
const uvec3 color_shift_vector = uvec3(16, 8, 0);

const uvec3 normal_shift_vector = uvec3(16, 1, 0);
const uvec3 normal_mask_vector  = uvec3(0xFFFF, 0x7FFF, 0x1);
///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////


vec3 hemioct_to_float32x3(vec2 e) {
//Rotate and scale the unit square back to the center diamond
vec2 temp = vec2(e.x + e.y, e.x - e.y) * 0.5;
vec3 v = vec3(temp, 1.0 - abs(temp.x) - abs(temp.y));
return normalize(v);
}


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


  uvec3 decoded_quantized_normal = (pos_14_13_13qz_col_8_8_8qz.zzz & normal_mask_vector) >> normal_shift_vector;
  vec3 decoded_hemioct_normal_components = vec3( vec2( ( (decoded_quantized_normal.xy / vec2(65536.0, 32768.0)) - 0.5) * 2.0), decoded_quantized_normal.z);


  vec3 decoded_normal = hemioct_to_float32x3(decoded_hemioct_normal_components.xy);

  //decoded_normal.xy   = ( (decoded_quantized_normal.xy / vec2(65536.0, 32768.0)) - 1.0) * 2.0;

  if(decoded_hemioct_normal_components.z < 0.5) {
    decoded_normal.z *= -1;
  }

 //decoded_normal.z  = sqrt( ( dot(decoded_normal.xy, decoded_normal.xy) - 1 ) ) * ( (decoded_normal.z >  0.5 ) ? 1.0 : -1.0)  ;

  //z_squared *= (decoded_quantized_normal.z == 1) ? 1.0 : -1.0;
  //gl_Position = kinect_mvp_matrix * vec4(unquantized_pos, 1.0);
  //VertexOut.color = 0.5 * (decoded_normal + 1.0 );//decoded_color; 
  VertexOut.color  = decoded_color;
  VertexOut.ms_pos = unquantized_pos;




  decoded_normal = normalize(decoded_normal);
  //decoded_normal = vec3(0.0, 0.0, 1.0);
  
  vec3 in_normal = decoded_normal;
  float in_radius = voxel_half_size;
  float radius_scaling = 1.0;

  @include "../common_SPOINTS/SPOINTS_vertex_pass_through.glsl"


  VertexOut.ms_normal = in_normal;

  //gl_PointSize = point_size*1.5;
  //gl_Position = vec4(gua_in_position_plus_packed_floatified_color.xyz, 1.0);

}
