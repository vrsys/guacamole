@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
//layout(location=0) in uvec2 pos_14_13_13qz_col_8_8_8qz;
//layout(location=0) in vec3 pos_3x32f;

@include "common/gua_camera_uniforms.glsl"
@material_uniforms@
@include "common/gua_vertex_shader_output.glsl"
@include "common/gua_global_variable_declaration.glsl"
@material_method_declarations_vert@

//out vec3 pass_color;

out vec2 pass_uvs;
//out vec3 pass_point_color;


//uniform uvec2 inv_xyz_volume_handles[4];
layout(binding=1) uniform sampler3D inv_xyz_volumes[4];
layout(binding=5) uniform sampler3D uv_volumes[4];

//uniform mat4 kinect_model_matrix;
uniform mat4 kinect_model_matrix;
uniform mat4 kinect_mv_matrix;
uniform mat4 kinect_mvp_matrix;

uniform mat4 inv_vol_to_world_matrix;

uniform float scaling_factor;

uniform int current_sensor_layer;

const vec3 conservative_bb_limit_min = vec3(-1.5, -0.5, -1.5);
const vec3 conservative_bb_limit_max = vec3( 1.5,  2.5,  1.5);

uniform vec3 tight_bb_min;
uniform vec3 tight_bb_max;

const vec3 quant_steps  = vec3(tight_bb_max - tight_bb_min) / (1<<16 - 1);




const uvec4 shift_vector = uvec4(18, 5, 0, 24);
const uvec4 mask_vector  = uvec4(0x3FFFu, 0x1FFFu, 0x1F, 0xFFu);

const uvec3 color_mask_vector  = uvec3(0xFF0000, 0xFF00, 0XFF);
const uvec3 color_shift_vector = uvec3(16, 8, 0);

const uvec2 uv_mask_vec  = uvec2(0xFFF000u, 0x000FFFu);
const uvec2 uv_shift_vec = uvec2(12u, 0u);


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////

uniform int texture_space_triangle_size;

#define VERTEX_COMPRESSION_TYPE_QUANTIZED

layout (std430, binding = 3) buffer Out_Sorted_Vertex_Tri_Data{
  uint[] in_sorted_vertex_pos_data;
};


vec2 viewport_offsets[4] = {{0.0, 0.0}, {0.5, 0.0}, {0.0, 0.5}, {0.5, 0.5}};

vec3 unquantize_uvec3_16u_to_vec3_32f(in uvec3 quantized_pos) {
  uvec3 compressed_pos = quantized_pos;

  vec3 bb_min = tight_bb_min;
  vec3 bb_max = tight_bb_max;
         
  vec3 bb_ranges = vec3(bb_max - bb_min);
  vec3 step_sizes = bb_ranges / ( (1<<16) - 1);
  vec3 unquantized_32_bit_pos = vec3(bb_min + (compressed_pos + vec3(0.5) ) * step_sizes);

  return unquantized_32_bit_pos;
}

vec3 get_unquantized_vertex_pos(in uint vertex_id) {
  const uint bit_size_of_vertex = 48;
  uint start_bit = bit_size_of_vertex * vertex_id;
  uint end_bit = start_bit + bit_size_of_vertex;

  bool is_even_index = (vertex_id % 2 == 0);

  
  uint first_int_read_idx = start_bit / 32;
  uint second_int_read_idx = (end_bit - 1) / 32;
  uint first_read_int = in_sorted_vertex_pos_data[first_int_read_idx];
  uint second_read_int = in_sorted_vertex_pos_data[second_int_read_idx];
  

  uvec3 quantized_pos = uvec3(0x00);
  if(is_even_index) {
    quantized_pos = uvec3(first_read_int >> 16, first_read_int & 0xFFFF, second_read_int >> 16);                                //0xFF00
  } else {
    quantized_pos = uvec3(first_read_int & 0xFFFF, second_read_int >> 16, second_read_int & 0xFFFF);  
  }
 

  vec3 unquantized_pos = unquantize_uvec3_16u_to_vec3_32f(quantized_pos);
  return unquantized_pos;
}


void main() {

  @material_input@
  

  vec3 unquantized_position = get_unquantized_vertex_pos(gl_VertexID);
  vec4 extracted_vertex_pos = vec4(unquantized_position,
                                   1.0);
  //vec4 extracted_vertex_pos = vec4(tri_positions[gl_VertexID % 3], 1.0);

  gua_world_position = (kinect_model_matrix * extracted_vertex_pos).xyz;
  gua_view_position  = (kinect_mv_matrix * extracted_vertex_pos).xyz;
  
  @material_method_calls_vert@
  @include "common/gua_varyings_assignment.glsl"

  gl_Position = kinect_mvp_matrix * extracted_vertex_pos;

  vec3 calib_sample_pos = (inv_vol_to_world_matrix * extracted_vertex_pos).xyz;

  vec3 pos_calib = texture(inv_xyz_volumes[current_sensor_layer], calib_sample_pos.xyz ).rgb;
  //vec3 pos_calib = texture(usampler3D(inv_xyz_volume_handles[current_sensor_layer]), calib_sample_pos.xyz ).rgb;
  vec2 pos_color = texture(uv_volumes[current_sensor_layer], pos_calib).xy / scaling_factor;

  pass_uvs = pos_color / 2.0 + viewport_offsets[current_sensor_layer];
}
