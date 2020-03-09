@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec2 plod_quad_coords;

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_global_variable_declaration.glsl"
///////////////////////////////////////////////////////////////////////////////
layout(binding=0) uniform sampler2D p02_color_texture;
layout(binding=1) uniform sampler2D p02_normal_texture;
layout(binding=2) uniform sampler2D p02_pbr_texture;
layout(binding=3) uniform sampler2D p02_weight_and_depth_texture;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_fragment_shader_output.glsl"


//info: for using standard hole filling, define USE_SS_HOLE_FILLING without MARK_FILLED_HOLES.
//      in order to visualize the filled holes, also enable MARK_FILLED_HOLES.

#define USE_SS_HOLE_FILLING
//#define MARK_FILLED_HOLES
///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  ivec2 current_fragment_pos = ivec2(gl_FragCoord.xy);

  vec2 accumulated_weight_and_depth = texelFetch(p02_weight_and_depth_texture, current_fragment_pos, 0).rg;
  float accumulated_weight = accumulated_weight_and_depth.x;

#ifdef USE_SS_HOLE_FILLING
  if(accumulated_weight == 0.0) {  //the pixel is a background pixel and we have to analyze, if we are able to fill it based on the neighbourhood

    vec2 neighbour_weights_and_depth[8]; 
    bool valid_neighbour_array[8];

    int bit_pattern = 0x0;

    uint cell_counter = 0;

    for( int y = -1; y <= 1; ++y ) {
      for( int x = -1; x <= 1; ++x ) {
        if(x == 0 && y == 0)
          continue;
        float current_accumulated_neighbour_weight = 0.0;
        ivec2 array_index_xy = ivec2(x, 
                                     y);

        neighbour_weights_and_depth[cell_counter] = texelFetch(p02_weight_and_depth_texture, current_fragment_pos + array_index_xy,0 ).rg;
        current_accumulated_neighbour_weight = neighbour_weights_and_depth[cell_counter].r;

        if(current_accumulated_neighbour_weight != 0.0) {
          bit_pattern |= (0x1 << cell_counter);
        }
        
        ++cell_counter;
      }
    }


    /* 8 background hole detection pattern after "An Image-space Approach to Interactive Point Cloud Rendering Including Shadows and Transparency" [Dobrev et al.]:

    3x3 box kernel:
    # F F   F F F   F F #   # # #   F F F   F F F   F # #  # # F   
    # B F   F B F   F B #   F B F   # B F   F B #   F B #  # B F
    # F F   # # #   F F #   F F F   # # F   F # #   F F F  F F F

    bit_index:
    0 1 2
    3   4
    5 6 7

    B = fackground (to be filled)
    F = foreground
    # = arbitrary

    */
    if(   ( (bit_pattern & 0xD6) == 0xD6) 
       || ( (bit_pattern & 0xF8) == 0xF8) 
       || ( (bit_pattern & 0x6B) == 0x6B)
       || ( (bit_pattern & 0x1F) == 0x1F)
       || ( (bit_pattern & 0x97) == 0x97)
       || ( (bit_pattern & 0xF4) == 0xF4)
       || ( (bit_pattern & 0xE9) == 0xE9)
       || ( (bit_pattern & 0x2F) == 0x2F)
      ) {   //matches, therefore perform filling
      
        uint num_accumulated_neighbours = 0;
        vec3 final_neighbours_color  = vec3(0.0, 0.0, 0.0);
        vec3 final_neighbours_normal = vec3(0.0, 0.0, 0.0);
        vec3 final_neighbours_pbr    = vec3(0.0, 0.0, 0.0);
        float final_neighbours_depth = 0.0;


    for(uint bit_index = 0; bit_index < 8; ++bit_index) {
      if( (bit_pattern & (0x1 << bit_index) ) != 0x0 ) { 
        ivec2 lookup_index_xy = ivec2(0, 0);

        if(bit_index == 0) {
          lookup_index_xy = ivec2(-1,-1);
        } else if (bit_index == 1) {
           lookup_index_xy = ivec2(0,-1);         
        } else if (bit_index == 2) {
           lookup_index_xy = ivec2(1,-1);
        } else if (bit_index == 3) {
           lookup_index_xy = ivec2( -1,0);         
        } else if (bit_index == 4) {
           lookup_index_xy = ivec2( 1, 0);   
        } else if (bit_index == 5) {
           lookup_index_xy = ivec2(-1, 1);         
        } else if (bit_index == 6) {
           lookup_index_xy = ivec2(0,1);   
        } else if (bit_index == 7) {
           lookup_index_xy = ivec2(1,1);   
        }

        vec2 current_neighbours_weight_and_depth = texelFetch(p02_weight_and_depth_texture, current_fragment_pos + lookup_index_xy,0 ).xy;

            ++num_accumulated_neighbours;
          #ifdef MARK_FILLED_HOLES
            final_neighbours_color  +=  vec3(0.0, 1.0, 1.0);   
            num_accumulated_neighbours = 1;     
          #else 
            final_neighbours_color  +=  (texelFetch(p02_color_texture, current_fragment_pos + lookup_index_xy,0 ).rgb) / current_neighbours_weight_and_depth.x;
            final_neighbours_normal +=  (texelFetch(p02_normal_texture, current_fragment_pos + lookup_index_xy,0 ).xyz) /  current_neighbours_weight_and_depth.x;
            final_neighbours_pbr    +=  (texelFetch(p02_pbr_texture, current_fragment_pos + lookup_index_xy,0 ).rgb) / current_neighbours_weight_and_depth.x;
            final_neighbours_depth  +=   current_neighbours_weight_and_depth.y / current_neighbours_weight_and_depth.x;
          #endif
        /*} else {  //this is an invalid case (pixel should become red)
         final_neighbours_color  +=  vec3(1.0, 0.0, 0.0);   
         num_accumulated_neighbours = 1;       
         break;w
        }*/

      }

    }


    final_neighbours_color  /= num_accumulated_neighbours;
    final_neighbours_normal /= num_accumulated_neighbours;
    final_neighbours_pbr    /= num_accumulated_neighbours;
    final_neighbours_depth  /= num_accumulated_neighbours;
 


    gua_color  = final_neighbours_color;
    gua_normal = final_neighbours_normal;
    gua_metalness  = final_neighbours_pbr.r;
    gua_roughness  = final_neighbours_pbr.g;
    gua_emissivity = final_neighbours_pbr.b;
    gua_alpha      = 1.0;
    gua_flags_passthrough = (gua_emissivity > 0.99999);

    gl_FragDepth = final_neighbours_depth;

    } else { //did not match, this background fragment will not contribute
      discard;
    }

  } else { 
#else
#endif
    float accumulated_depth =  accumulated_weight_and_depth.y ;
    float blended_depth = accumulated_depth / accumulated_weight;

    vec3  normalized_color  = vec3(1.0);
    float output_depth  = 1.0;
    vec3  output_normal = vec3(0.0);
    vec3 coords = vec3(plod_quad_coords, 0.0);
    //vec3 accumulated_color = texture(p02_color_texture, coords.xy).rgb;
    vec3 accumulated_color = texelFetch(p02_color_texture, current_fragment_pos, 0).rgb;
   

    normalized_color = accumulated_color.rgb / accumulated_weight ;
    //normalized_color = accumulated_color.rgb;
    normalized_color = normalized_color;
   
    vec3 accumulated_normal = texelFetch(p02_normal_texture, current_fragment_pos, 0).rgb;
    vec3 normalized_normal = normalize(accumulated_normal.rgb / accumulated_weight );
    //vec3 normalized_normal = accumulated_normal.rgb / accumulated_weight;


    //float depth_visibility_pass = texture2D( p01_log_depth_texture, coords.xy).r;
/*
    if(normalized_normal.z < 0) {
      normalized_normal *= -1;
    }
*/
    gua_color = normalized_color.rgb;
    gua_normal = normalized_normal;
    //gua_normal = vec3(1.0, 0.0, 0.0);

    vec3 written_pbr_coeffs = (texelFetch(p02_pbr_texture, current_fragment_pos, 0).rgb) / accumulated_weight;

    gua_metalness  = written_pbr_coeffs.r;
    gua_roughness  = written_pbr_coeffs.g;
    gua_emissivity = written_pbr_coeffs.b;
    gua_alpha      = 1.0;
    gua_flags_passthrough = (gua_emissivity > 0.99999);

    // calculate world position from blended depth
    //vec4 world_pos_h = gua_inverse_projection_view_matrix * vec4(gl_FragCoord.xy, depth_visibility_pass, 1.0);

    if( !(blended_depth >= 0.0 && blended_depth  <= 0.9999999) )
      blended_depth = 0.0;

    vec4 world_pos_h = gua_inverse_projection_view_matrix * vec4(gl_FragCoord.xy, blended_depth, 1.0);
    gua_world_position = world_pos_h.xyz / world_pos_h.w;


    gl_FragDepth = blended_depth; 

#ifdef USE_SS_HOLE_FILLING
  }
#else
#endif


  //gua_color = texelFetch(p02_normal_texture, current_fragment_pos ,0 ).xyz;
  //gua_normal = vec3(0.0, 1.0, 0.0);
  @include "common/gua_write_gbuffer.glsl"



}

