@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
layout(location=0) in uvec2 pos_14_13_13qz_col_8_8_8qz;

@include "common/gua_camera_uniforms.glsl"
@material_uniforms@
@include "common/gua_vertex_shader_output.glsl"
@include "common/gua_global_variable_declaration.glsl"
@material_method_declarations_vert@

//out vec3 pass_color;

out vec2 pass_uvs;
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

const uvec2 uv_mask_vec  = uvec2(0xFFF000u, 0x000FFFu);
const uvec2 uv_shift_vec = uvec2(12u, 0u);

//const uvec3 normal_shift_vector = uvec3(16, 1, 0);
//const uvec3 normal_mask_vector  = uvec3(0xFFFF, 0x7FFF, 0x1);
///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////

#define ONE_D_TEXTURE_ATLAS_SIZE 2048

uniform int texture_space_triangle_size;


void main() {
  uvec4 masked_and_shifted_pos = (uvec4(pos_14_13_13qz_col_8_8_8qz.xxx, pos_14_13_13qz_col_8_8_8qz.y) >> shift_vector) & mask_vector;
  uvec3 decoded_quantized_pos  = uvec3(masked_and_shifted_pos.xy, masked_and_shifted_pos.z | (masked_and_shifted_pos.w << 5) );
  vec3 unquantized_pos = conservative_bb_limit_min + decoded_quantized_pos * quant_steps;

  // reserve unique id for the new face
  uint triangle_write_index = gl_VertexID / 3;


  uint triangle_vertex_base_offset = 3 * triangle_write_index;




    uvec2 texture_atlas_size = uvec2(ONE_D_TEXTURE_ATLAS_SIZE, ONE_D_TEXTURE_ATLAS_SIZE);
    uvec2 triangle_size_in_pixel = uvec2(texture_space_triangle_size, texture_space_triangle_size);

    uvec2 num_quads_per_axis = (texture_atlas_size / uvec2(triangle_size_in_pixel.x + 1, triangle_size_in_pixel.y) ) ;

    // two consecutive indices should be pinned to the same position with two of its corners. the third corner needs to be flipped
    // therefore the quad index is the same for two consecutie triangles
    uint one_d_quad_index = triangle_write_index / 2;

    uvec2 two_d_quad_index = uvec2(one_d_quad_index % num_quads_per_axis.x, one_d_quad_index / num_quads_per_axis.x );

    //uvec2 tri_first_corner_row_col_quad_pos = uvec2(one_d_quad_index % (num_quads_per_axis.x * 2), one_d_quad_index / num_quads_per_axis.x );

    //uvec2 triangle_corner_bias = uvec2(0, 0) * ((first_corner_row_col_quad_pos)); 
    //uvec2 first_corner_offset  = uvec2(1, 1) * (triangle_write_index % 2);

    uvec2 rasterization_offset = uvec2(1, 1);//first_corner_row_col_quad_pos;// + (triangle_write_index % 2);

    uvec2 integer_uv_coords_uncompressed;

    bool is_even_triangle_index = ( 0 == triangle_write_index % 2 );


    vec2 uv_shift = vec2(0.5);
    if(is_even_triangle_index) {
    /*/  if(0 == gl_VertexID%3)  {
        uv_shift = vec2(0.5, -0.5);
      }*/
      if(1 == gl_VertexID % 3)  {
        //uv_shift = vec2(-1.0, 0.0);
      }
      if(2 == gl_VertexID%3)  {
        //uv_shift = vec2(0.0, 1.0);
      }  
    } else {

    }

 vec2 decoded_uvs =  (vec2(uvec2(pos_14_13_13qz_col_8_8_8qz.yy & uv_mask_vec ) >> uv_shift_vec) ) / vec2(ONE_D_TEXTURE_ATLAS_SIZE) 
                       + (uv_shift / vec2(ONE_D_TEXTURE_ATLAS_SIZE)) ;
  

  pass_uvs = decoded_uvs;
  gl_Position = kinect_mvp_matrix * vec4(unquantized_pos, 1.0);


}
