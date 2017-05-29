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

layout(binding=0) uniform @gua_tv_3_sampler_3d_type@ volume_texture;
layout(binding=1) uniform sampler2D codebook_texture;

uniform vec4 ms_eye_pos;

uniform float gua_texel_width;
uniform float gua_texel_height;

uniform float iso_value = 0.5;

uniform int num_codewords_per_row = 0;

uniform ivec3 block_offset_vector = ivec3(0, 0, 0);
uniform int total_block_size = 0;
uniform ivec3 volume_dimensions = ivec3(0, 0, 0);

vec3 ms_shading_pos = vec3(0.0, 0.0, 0.0);

@material_uniforms@
@material_method_declarations_frag@


//float mod289(float x){return x - floor(x * (1.0 / 289.0)) * 289.0;}
/*
vec4 mod289(vec4 x){return x - floor(x * (1.0 / 289.0)) * 289.0;}
vec4 perm(vec4 x){return mod289(((x * 34.0) + 1.0) * x);}

float noise(vec3 p){
    vec3 a = floor(p);
    vec3 d = p - a;
    d = d * d * (3.0 - 2.0 * d);

    vec4 b = a.xxyy + vec4(0.0, 1.0, 0.0, 1.0);
    vec4 k1 = perm(b.xyxy);
    vec4 k2 = perm(k1.xyxy + b.zzww);

    vec4 c = k2 + a.zzzz;
    vec4 k3 = perm(c);
    vec4 k4 = perm(c + 1.0);

    vec4 o1 = fract(k3 * (1.0 / 41.0));
    vec4 o2 = fract(k4 * (1.0 / 41.0));

    vec4 o3 = o2 * d.z + o1 * (1.0 - d.z);
    vec2 o4 = o3.yw * d.x + o3.xz * (1.0 - d.x);

    return o4.y * d.y + o4.x * (1.0 - d.y);
}
*/
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

float step_size = 0.0004372;



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


float get_mode_independent_sample(vec3 current_pos) {
  //return texture(volume_texture, current_pos ).r;


  return get_uninterpolated_sample_SW_VQ(current_pos, volume_texture, codebook_texture);
  return get_trilinearly_interpolated_sample_SW_VQ(current_pos, volume_texture, codebook_texture);

}


const vec3 voxel_dimensions = vec3(1.0, 1.0, 1.0) / textureSize(volume_texture, 0).xyz;

vec3 get_gradient(vec3 in_sampling_pos) {

  vec3 gradient;
  gradient.x = 0.5*( get_mode_independent_sample(in_sampling_pos + vec3(voxel_dimensions.x, 0.0, 0.0)) - get_mode_independent_sample(in_sampling_pos - vec3(voxel_dimensions.x, 0.0, 0.0) )   );
  gradient.y = 0.5*( get_mode_independent_sample(in_sampling_pos + vec3(0.0, voxel_dimensions.y, 0.0)) - get_mode_independent_sample(in_sampling_pos - vec3(0.0, voxel_dimensions.y, 0.0) )   );
  gradient.z = 0.5*( get_mode_independent_sample(in_sampling_pos + vec3(0.0, 0.0, voxel_dimensions.z)) - get_mode_independent_sample(in_sampling_pos - vec3(0.0, 0.0, voxel_dimensions.z) )   );
    

  return -normalize(gradient);
}

vec3 raycast_iso_surface() {
  vec3 ray_origin    = FragmentIn.pos_ms;
  //vec3 ray_direction = normalize(FragmentIn.pos_ms - ms_eye_pos.xyz);
  vec3 ray_direction = normalize(FragmentIn.pos_ms - ms_eye_pos.xyz);

  //vec2 t_min_max = intersect_ray_with_unit_box(ray_origin, ray_direction);

  vec3 ray_increment = ray_direction * step_size;

  vec3 current_pos = ray_origin;

  vec3 first_pos = vec3(0.0, 0.0, 0.0);
  vec3 red = vec3(0.0, 0.0, 0.0);
  /*
  if(t_min_max[0] < 0) {
  current_pos = ms_eye_pos.xyz;
    red = vec3(1.0,0.0,0.0);
  } else {
  current_pos = ray_origin + t_min_max[0] * ray_direction;

  }*/

  current_pos += ray_increment;
/*
  //plane intersection
  compute_ray_plane_intersection(vec3(0.5, 0.5, 0.5), vec3(0.0, 0.0, 0.5),
                                 current_pos, cam_to_box_end_vec,
                                 t_min_max[0], t_min_max[1]);
*/
  //current_pos = current_pos + t_min_max[0] * cam_to_box_end_vec;

 // current_pos += ray_increment;

  vec3 previous_pos = current_pos;
  float previous_sample = 0;

  float num_samples_taken = 0;

  bool found_iso_value = false;
  bool is_smaller_than_iso = true;
  bool was_smaller_than_iso = is_smaller_than_iso;

  while(is_inside_vol(current_pos) ) {

    float density = get_mode_independent_sample(current_pos);
    
      if( sign(density - iso_value) != sign(previous_sample - iso_value) ) {
        vec3 min = previous_pos;
        vec3 max = current_pos;
        vec3 mid;
        int iterations = 0;

        for(int i = 0; i < 10; ++i) {
        //while (length(max - min) > 0.0000001 && ++iterations < 20) {
          if(length(max-min) <= 0.0000001 ) {
            break;
          }
          mid = (min + max) * 0.5;
          float s_mid = get_mode_independent_sample(current_pos);
          if (s_mid == iso_value) {
            break;
          }
          if (s_mid > iso_value) {
            max = mid;
          }
          else{
            min = mid;
          }

        }
        current_pos = mid;
        density = get_mode_independent_sample(current_pos);

        found_iso_value = true;
        break;
      }


    previous_sample = density;
    previous_pos = current_pos;
    current_pos +=  ray_increment;
    ++num_samples_taken;

  }

  if(false == found_iso_value) {
    discard;
  }

  return current_pos;
};

void main() {

  ms_shading_pos = raycast_iso_surface();

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

  vec4 projected_pos = gua_model_view_projection_matrix * vec4(ms_shading_pos, 1);
  projected_pos /= projected_pos.w;

  float gbuffer_depth = projected_pos.z * 0.5 + 0.5;
  gl_FragDepth = gbuffer_depth;
  submit_fragment(gbuffer_depth);
}