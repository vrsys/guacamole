@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_fragment_shader_output.glsl"
@include "common/gua_global_variable_declaration.glsl"
@include "common/gua_abuffer_collect.glsl"
///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in Data {
  vec3 pos_ms;
} FragmentIn;

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////

#if @gua_tv_3_uncompressed@
layout(binding=0) uniform sampler3D volume_texture;
#endif
#if @gua_tv_3_vq_compressed@
layout(binding=0) uniform usampler3D volume_texture;
layout(binding=1) uniform sampler2D codebook_texture;
#endif

#if @gua_tv_3_volume_behaviour@

layout(binding=2) uniform sampler2D compositing_color_texture;
layout(binding=3) uniform sampler2D compositing_auxiliary_depth_buffer;



layout(location = 0) out vec4 out_color;

uniform int vol_idx;
#endif

///////////////////////////////////////////////////////////////////////////////
// common ray casting uniforms
///////////////////////////////////////////////////////////////////////////////

uniform vec4 ms_eye_pos;
uniform int num_codewords_per_row = 0;
uniform ivec3 block_offset_vector = ivec3(0, 0, 0);
uniform int total_block_size = 0;
uniform ivec3 volume_dimensions = ivec3(0, 0, 0);


uniform float iso_value = 0.5;


vec3 ms_shading_pos = vec3(0.0, 0.0, 0.0);

@material_uniforms@
@material_method_declarations_frag@



void compute_ray_plane_intersection(in vec3 ms_plane_pos, in vec3 ms_plane_normal,
	                                in vec3 ms_ray_pos, in vec3 in_ms_ray_direction,
	                                in out float min_t, in out float max_t) {
   // assuming vectors are all normalized

  vec3 normalized_plane_normal = normalize(ms_plane_normal);

  float denom = dot(normalized_plane_normal, in_ms_ray_direction);


  float t = 0.0;
  if ( (denom > 0.0 && denom > 1e-10) ) {
    vec3 line_pos_to_plane_pos = ms_plane_pos - ms_ray_pos;
    t = dot(line_pos_to_plane_pos, normalized_plane_normal) / denom;
  }

  if(t > 0.0 && min_t < t) {
  	min_t = t;
  }

}

bool is_inside_vol(vec3 pos) {
  return all(greaterThanEqual(pos, vec3(0.0) )) 
      && all(lessThanEqual(pos, vec3(1.0) ) );
}

vec2 intersect_ray_with_unit_box(in vec3 origin, in vec3 direction ) {
  vec2 t_min_max = vec2(0.0, 0.0);

    vec3 inv_direction = 1.0 / direction;

    int sign[3];

    if(inv_direction.x < 0) {
      sign[0] = 1;
    } else {
      sign[0] = 0;
    }
    if(inv_direction.y < 0) {
      sign[1] = 1;
    } else {
      sign[1] = 0;
    }
    if(inv_direction.z < 0) {
      sign[2] = 1;
    } else {
      sign[2] = 0;
    }

    vec3 aabb[2];

    for(int dim_idx = 0; dim_idx < 3; ++dim_idx) {
      aabb[0][dim_idx] = 0.0;
      aabb[1][dim_idx] = 1.0;   
    }

    vec2 ty_min_max;
    vec2 tz_min_max;
    float tymin, tymax, tzmin, tzmax;
    t_min_max[0] = inv_direction.x * (aabb[sign[0]].x - origin.x);
    t_min_max[1] = inv_direction.x * (aabb[1-sign[0]].x - origin.x);
    ty_min_max[0] = inv_direction.y * (aabb[sign[1]].y - origin.y);
    ty_min_max[1] = inv_direction.y * (aabb[1-sign[1]].y - origin.y);
    tz_min_max[0] = inv_direction.z * (aabb[sign[2]].z - origin.z);
    tz_min_max[1] = inv_direction.z * (aabb[1-sign[2]].z - origin.z);
    t_min_max[0] = max(max(t_min_max[0], ty_min_max[0]), tz_min_max[0]);
    t_min_max[1] = min(min(t_min_max[1], ty_min_max[1]), tz_min_max[1]);

    return t_min_max;
}

float step_size = 0.001372;


#if @gua_tv_3_vq_compressed@
float get_uninterpolated_sample_SW_VQ(vec3 in_pos, usampler3D in_index_texture, sampler2D in_codebook) {

  uint one_d_idx = texture(volume_texture, in_pos ).r;

  ivec3 col_additions = ivec3( in_pos * volume_dimensions ) %  block_offset_vector.y;
  float col = dot(col_additions, block_offset_vector);

  uint col_offset = (one_d_idx % num_codewords_per_row) * total_block_size;
  uint row = one_d_idx / num_codewords_per_row;

  float density = texelFetch( codebook_texture, ivec2(col_offset + uint(col), row), 0 ).r;

  return density;
}

float get_trilinearly_interpolated_sample_SW_VQ(vec3 in_pos, usampler3D in_index_texture, sampler2D in_codebook) {

  const vec3 volume_voxel_distance = (1.0/ (volume_dimensions ) );
  const vec3 half_volume_voxel_distance = volume_voxel_distance / 2.0;
  const vec3 lower_bound = vec3( half_volume_voxel_distance );
  const vec3 upper_bound = vec3(1.0 - half_volume_voxel_distance);


  vec3 pos_to_use = in_pos;

  vec3 normalized_pos = ( pos_to_use / ( volume_voxel_distance ) ) / (volume_dimensions);

  vec3 floor_positions =    ( normalized_pos  - half_volume_voxel_distance) ; 
  vec3 ceil_positions  =  ( normalized_pos  + half_volume_voxel_distance) ;

  float blf = get_uninterpolated_sample_SW_VQ(floor_positions, in_index_texture, in_codebook);
  float brf = get_uninterpolated_sample_SW_VQ(vec3(ceil_positions.x, floor_positions.yz), in_index_texture, in_codebook);
  float tlf = get_uninterpolated_sample_SW_VQ(vec3(floor_positions.x, ceil_positions.y, floor_positions.z), in_index_texture, in_codebook);
  float trf = get_uninterpolated_sample_SW_VQ(vec3(ceil_positions.xy, floor_positions.z), in_index_texture, in_codebook);

  float blb = get_uninterpolated_sample_SW_VQ(vec3(floor_positions.xy, ceil_positions.z), in_index_texture, in_codebook);
  float brb = get_uninterpolated_sample_SW_VQ(vec3(ceil_positions.x, floor_positions.y, ceil_positions.z), in_index_texture, in_codebook);
  float tlb = get_uninterpolated_sample_SW_VQ(vec3(floor_positions.x, ceil_positions.yz), in_index_texture, in_codebook);
  float trb = get_uninterpolated_sample_SW_VQ(ceil_positions, in_index_texture, in_codebook);
  
  vec3 interpolation_weights = mod( (pos_to_use + half_volume_voxel_distance) / (volume_voxel_distance), 1.0);

  vec4 left_samples  = vec4(blf, blb, tlf, tlb);
  vec4 right_samples = vec4(brf, brb, trf, trb);

  vec4 x_interpolated = mix(left_samples, right_samples, interpolation_weights.x);
  
  vec2 bottom_samples = x_interpolated.xy;
  vec2 top_samples    = x_interpolated.zw;

  vec2 y_interpolated = mix(bottom_samples, top_samples, interpolation_weights.y);

  return mix(y_interpolated.x, y_interpolated.y, interpolation_weights.z);
}
#endif

#if @gua_tv_3_uncompressed@
float get_uncompressed_sample(vec3 current_pos) {
  return texture(volume_texture, current_pos ).r;
}
#endif

float get_mode_independent_sample(vec3 current_pos) {
  //return texture(volume_texture, current_pos ).r;

  #if @gua_tv_3_uncompressed@
    return get_uncompressed_sample(current_pos);
  #endif

  #if @gua_tv_3_vq_compressed@
    #if @gua_tv_3_spatially_nearest_filter@
      return get_uninterpolated_sample_SW_VQ(current_pos, volume_texture, codebook_texture);
    #endif
    #if @gua_tv_3_spatially_linear_filter@
      return get_trilinearly_interpolated_sample_SW_VQ(current_pos, volume_texture, codebook_texture);
    #endif
  #endif
  //return get_trilinearly_interpolated_sample_SW_VQ(current_pos, volume_texture, codebook_texture);

}


const vec3 voxel_dimensions = vec3(1.0, 1.0, 1.0) / textureSize(volume_texture, 0).xyz;

vec3 get_gradient(vec3 in_sampling_pos) {

  vec3 gradient;
  gradient.x = 0.5*( get_mode_independent_sample(in_sampling_pos + vec3(voxel_dimensions.x, 0.0, 0.0)) - get_mode_independent_sample(in_sampling_pos - vec3(voxel_dimensions.x, 0.0, 0.0) )   );
  gradient.y = 0.5*( get_mode_independent_sample(in_sampling_pos + vec3(0.0, voxel_dimensions.y, 0.0)) - get_mode_independent_sample(in_sampling_pos - vec3(0.0, voxel_dimensions.y, 0.0) )   );
  gradient.z = 0.5*( get_mode_independent_sample(in_sampling_pos + vec3(0.0, 0.0, voxel_dimensions.z)) - get_mode_independent_sample(in_sampling_pos - vec3(0.0, 0.0, voxel_dimensions.z) )   );
    

  return -normalize(gradient);
}

#if @gua_tv_3_surface_pbr_behaviour@
@include "_mode__isosurface_ray_casting.frag"
#endif

#if @gua_tv_3_volume_behaviour@
  #if @gua_tv_3_mode_vol_max_intensity@
  @include "_mode__max_intensity_ray_casting.frag"
  #endif

  #if @gua_tv_3_mode_vol_avg_intensity@
  @include "_mode__avg_intensity_ray_casting.frag"  
  #endif

  #if @gua_tv_3_mode_vol_compositing@
  @include "_mode__compositing_ray_casting.frag"  
  #endif

  #if @gua_tv_3_mode_vol_isosurface@
  @include "_mode__isosurface_ray_casting.frag" 
  #endif
#endif


void main() {

#if @gua_tv_3_surface_pbr_behaviour@
  ms_shading_pos = raycast_isosurface();
#endif



  //these two variables should not be altered by any material programs
  gua_normal     = vec3( (gua_normal_matrix * vec4(get_gradient(ms_shading_pos),1.0) ).xyz );
  gua_world_position = (gua_model_matrix * vec4(ms_shading_pos, 1.0)).xyz;

  

 /// if( 0 != one_d_idx % 2) {
   gua_color      = vec3(1.0, 1.0, 1.0);
   gua_alpha      = 1.0;
   gua_metalness  = 0.0;
   gua_roughness  = 1.0;
   gua_emissivity = 0.0;   
  //} else {
    /*
    gua_color      = vec3(1.0, 0.0, 0.0);
    gua_alpha      = 1.0;
    gua_metalness  = 0.0;
    gua_roughness  = 1.0;
    gua_emissivity = 0.0;  
  }
*/


  //float density = texture(codebook_texture,);


//  gua_color = vec3(density, density, density);

  //these variables can freely be altered by material programs
  gua_color      = vec3(1.0, 1.0, 1.0);
  gua_alpha      = 1.0;
  gua_metalness  = 0.0;
  gua_roughness  = 1.0;
  gua_emissivity = 0.0;


  @material_input@
  if (gua_rendering_mode != 1) {
    @material_method_calls_frag@
  }

#if @gua_tv_3_volume_behaviour@

  vec4 current_pass_color = vec4(0.0);
  #if @gua_tv_3_mode_vol_max_intensity@
  float max_intensity = 0.0;
  ms_shading_pos = raycast_max_intensity(max_intensity);
  current_pass_color = max_intensity * vec4(gua_color*gua_alpha, gua_alpha);
  #endif

  #if @gua_tv_3_mode_vol_avg_intensity@
  float avg_intensity = 0.0;
  ms_shading_pos = raycast_avg_intensity(avg_intensity);
  current_pass_color = avg_intensity * vec4(gua_color*gua_alpha, gua_alpha);
  #endif

  #if @gua_tv_3_mode_vol_compositing@
  vec4 composited_color = vec4(0.0);
  ms_shading_pos = raycast_compositing(composited_color);
  current_pass_color = composited_color;
  #endif

  #if @gua_tv_3_mode_vol_isosurface@
  ms_shading_pos = raycast_isosurface();
  current_pass_color = vec4(gua_color, 1.0);
  #endif

  //current_pass_color is the color that is created by the current volume rendering pass.
  //previous color is the color which is in the front buffer texture

  vec4 screen_space_cube_fragment_pos = gua_model_view_projection_matrix * vec4(FragmentIn.pos_ms, 1);
  screen_space_cube_fragment_pos /= screen_space_cube_fragment_pos.w;

  float current_pass_depth = screen_space_cube_fragment_pos.z * 0.5 + 0.5;


  vec4 previous_color = texelFetch(compositing_color_texture, ivec2(gl_FragCoord.xy), 0);
  float previous_depth = texelFetch(compositing_auxiliary_depth_buffer, ivec2(gl_FragCoord.xy), 0).r;

  vec4 src = vec4(0);
  vec4 dst = vec4(0);
/*
  if(previous_depth < current_pass_depth) {
    src = previous_color;
    dst = current_pass_color;
  } else {
    src = current_pass_color;
    dst = previous_color;

    //overwrite frontmost depth value
    gl_FragDepth = current_pass_depth;
  }
*/
  float omda_sa = (1.0 - dst.a) * 1.0; //the 1.0 is the src.a, which we can only assume to be 1

  dst.rgb += omda_sa * src.rgb;
  dst.a   += omda_sa;

  out_color = dst.rgba;

  out_color = current_pass_color + previous_color;

 //out_color = current_pass_color + vec4(0.0, 0.0, 0.0, 0.3*vol_idx);




  //out_color = vec4(vol_idx, 0.0, 0.0, 1.0);
  //float composited_alpha = (1.0 - fetched_color.a) * back_blending_color.a;
  //out_color = vec4(texelFetch(front_to_back_blending_texture, ivec2(gl_FragCoord.xy), 0).rgb * (1.0-back_blending_color.a) + back_blending_color.rgb * back_blending_color.a, composited_alpha);
#endif

#if @gua_tv_3_surface_pbr_behaviour@
  vec4 projected_pos = gua_model_view_projection_matrix * vec4(ms_shading_pos, 1);
  projected_pos /= projected_pos.w;

  float gbuffer_depth = projected_pos.z * 0.5 + 0.5;
  gl_FragDepth = gbuffer_depth;


  submit_fragment(gbuffer_depth);
#endif


}