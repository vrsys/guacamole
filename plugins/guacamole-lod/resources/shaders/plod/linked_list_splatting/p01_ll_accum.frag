@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"


///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in VertexData {
  vec2 pass_uv_coords;
  float es_depth;
  flat uint pass_further_global_surfel_id;
  vec3 pass_world_position;
} VertexIn;

uniform float EPSILON;
uniform int num_blended_frags;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

//surfel attribute images
layout(binding = 0, RGBA8) writeonly uniform restrict image2D out_surfels_pbr;
layout(binding = 1, RGBA8) writeonly uniform restrict image2D out_surfels_normal;

//linked list images
layout(binding = 2, RGBA16UI) coherent uniform restrict uimageBuffer linked_list_buffer;
layout(binding = 3, R32UI) coherent uniform restrict uimage2D fragment_count_img;

//depth buffer substitution image
layout(binding = 4, R32UI) coherent uniform restrict uimage2D min_es_distance_image;

//#define EPSILON 0.001
#define MAX_INT_32 2147483647
#define NUM_GAUSSIAN_BLEND_STEPS 20.0


uint uintify_float_depth(in float float_depth) {
  return uint( MAX_INT_32 - ( MAX_INT_32 * (float_depth - gua_clip_near) / (gua_clip_far - gua_clip_near) ) );
} 

float floatify_uint_depth(in uint uint_depth) {
  return float( ( float( MAX_INT_32 - uint_depth) / float(MAX_INT_32) ) * (gua_clip_far-gua_clip_near) + gua_clip_near);
}

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  if( dot(VertexIn.pass_uv_coords, VertexIn.pass_uv_coords) > 1)
    discard;

  //clip against global clipping planes
  vec3 gua_world_position = VertexIn.pass_world_position;

  for (int i=0; i < gua_clipping_plane_count; ++i) {

    if (dot(gua_clipping_planes[i].xyz, gua_world_position.xyz) + gua_clipping_planes[i].w < 0) {
      discard;
    }
  }


  float own_depth = VertexIn.es_depth;
  uint  uintified_own_depth = uintify_float_depth(own_depth);

  uint current_min_depth = imageAtomicMax(min_es_distance_image, ivec2(gl_FragCoord.xy), uintified_own_depth).x;

  float floatified_min_depth = floatify_uint_depth(current_min_depth);



  //TODO replace 0.5 by some model dependent parameter (max radius or so)
  if(own_depth > floatified_min_depth + 0.003) {
    discard;
  }

  
  uint global_surfel_id = VertexIn.pass_further_global_surfel_id;


  uint weight_index = uint(floor(length(VertexIn.pass_uv_coords) * NUM_GAUSSIAN_BLEND_STEPS ));

  
  uint undecomposed_converted_depth = floatBitsToUint(VertexIn.es_depth);
  //uint undecomposed_converted_depth = floatBitsToUint(0);
  uvec4 insert_into_list = uvec4(undecomposed_converted_depth & 0x0000FFFF, 
                                (undecomposed_converted_depth >> 16) & 0x0000FFFF,
                                global_surfel_id & 0x0000FFFF, 
                                ( (global_surfel_id >> 16) & 0x000007FF ) |  ( (weight_index ) << 11 ) );


  uint pixel_chunk_start = (uint(gl_FragCoord.x)  +  uint(gl_FragCoord.y) * gua_resolution.x) * num_blended_frags;

  uint currently_written_fragments = imageAtomicAdd(fragment_count_img, ivec2(gl_FragCoord.xy), uint(1) );

  bool already_done = false;

  uint write_offset = 0;

  //imageStore(min_es_distance_image, ivec2(gl_FragCoord.xy), uvec4(200, 200, 200, 200) );

  uint local_offset = pixel_chunk_start;
  if(currently_written_fragments < num_blended_frags) {

    write_offset = currently_written_fragments;
    uint new_idx = pixel_chunk_start + write_offset;

    imageStore(linked_list_buffer, int(new_idx), insert_into_list);
    discard;
  } else {
    discard;
  }

}

