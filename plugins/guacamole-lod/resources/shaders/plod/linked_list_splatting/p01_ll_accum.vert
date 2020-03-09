@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

@material_uniforms@

// input attributms
layout(location = 0) in vec3 in_position;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;

//surfel attribute images
layout(binding = 0, RGBA8) writeonly uniform restrict image2D out_surfels_pbr;
layout(binding = 1, RGBA8) writeonly uniform restrict image2D out_surfels_normal;

//linked list images
layout(binding = 2, RGBA16UI) coherent uniform restrict uimageBuffer linked_list_buffer;
layout(binding = 3, R32UI) coherent uniform restrict uimage2D fragment_count_img;

//depth buffer substitution image
layout(binding = 4, R32UI) coherent uniform restrict uimage2D min_es_distance_image;

uniform mat4 inverse_transpose_model_view_matrix;
uniform float radius_scaling;
uniform float max_surfel_radius;
uniform bool enable_backface_culling;

out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  flat uint pass_global_surfel_id;
} VertexOut;

void main() {

  @include "../common_LOD/PLOD_vertex_pass_through.glsl"
  
  //compute unique surfel index in attribute textures and write attributes
  uint image_width = imageSize(out_surfels_pbr).x;
  uint global_surfel_idx = gl_VertexID;
  uint surfel_image_idx_x = global_surfel_idx % image_width;
  uint surfel_image_idx_y =  int(global_surfel_idx / image_width) ;
  ivec2 surfel_pos = ivec2(surfel_image_idx_x, surfel_image_idx_y);

  float gua_metalness  = 0.0;
  float gua_roughness  = 1.0;
  float gua_emissivity = 1.0; // pass through if unshaded

  imageStore(out_surfels_pbr, surfel_pos, vec4(gua_metalness, gua_roughness, gua_emissivity, 1.0) );

  vec4 world_normal = gua_normal_matrix * vec4(in_normal, 0.0);
  vec4 view_normal  = inverse_transpose_model_view_matrix * vec4(in_normal, 0.0);

  if (enable_backface_culling && view_normal.z < 0.0) {
    VertexOut.pass_ms_u = vec3(0.0);
    VertexOut.pass_ms_v = vec3(0.0);  
  } else {
    float flip_normal = 1.0 - 2.0 * float(view_normal.z < 0.0);
    world_normal = flip_normal * world_normal;
  }

  imageStore(out_surfels_normal, surfel_pos, vec4( normalize(world_normal.xyz), 0.0 ) );

  VertexOut.pass_global_surfel_id = global_surfel_idx;

}