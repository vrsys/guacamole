@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in Data {
  vec3 pos_ms;
} FragmentIn;

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_global_variable_declaration.glsl"

layout(binding=0) uniform sampler3D volume_texture;

uniform vec4 ms_eye_pos;

uniform float gua_texel_width;
uniform float gua_texel_height;

uniform float iso_value = 0.5;
@include "common/gua_fragment_shader_output.glsl"

//layout(location = 0) out vec4 out_color;

//returns model space t (where t=0.0 old starting point & t= 1.0)
//front and back orientation need to be distinguished



float mod289(float x){return x - floor(x * (1.0 / 289.0)) * 289.0;}
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

float step_size = 0.001;

float get_mode_independent_sample(vec3 current_pos) {
  return texture(volume_texture, current_pos ).r;
}


const vec3 voxel_dimensions = vec3(1.0, 1.0, 1.0) / textureSize(volume_texture, 0).xyz;

vec3 get_gradient(vec3 in_sampling_pos) {

  vec3 gradient;
  gradient.x = 0.5*( get_mode_independent_sample(in_sampling_pos + vec3(voxel_dimensions.x, 0.0, 0.0)) - get_mode_independent_sample(in_sampling_pos - vec3(voxel_dimensions.x, 0.0, 0.0) )   );
  gradient.y = 0.5*( get_mode_independent_sample(in_sampling_pos + vec3(0.0, voxel_dimensions.y, 0.0)) - get_mode_independent_sample(in_sampling_pos - vec3(0.0, voxel_dimensions.y, 0.0) )   );
  gradient.z = 0.5*( get_mode_independent_sample(in_sampling_pos + vec3(0.0, 0.0, voxel_dimensions.z)) - get_mode_independent_sample(in_sampling_pos - vec3(0.0, 0.0, voxel_dimensions.z) )   );
    

  return -normalize(gradient);
}


void main() {

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

        for(int i = 0; i < 7; ++i) {
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
    //get_gradient(current_pos);
    //out_color = vec4(1.0, 1.0, 1.0, 1.0);
    //out_color = vec4( (get_gradient(current_pos) + 1.0) / 2.0, 1.0 );

    //gua_color = vec3((get_gradient(current_pos) + 1.0) / 2.0);
  }
 
  //the final pos is now stored in current pos

  vec4 projected_pos = gua_model_view_projection_matrix * vec4(current_pos, 1);
       projected_pos /= projected_pos.w;


  gl_FragDepth = projected_pos.z * 0.5 + 0.5;
  gua_color      = vec3(0.8,0.8, 0.8);//0.5 * vec3(0.0, 0.68235294, 0.3372549019) + 0.5 * noise(current_pos*100.0) * vec3(0.0, 0.68235294, 0.3372549019);
  gua_normal     = vec3( (gua_normal_matrix * vec4(get_gradient(current_pos),1.0) ).xyz );// * 0.5 + 0.5;//normalize((gua_normal_matrix * vec4(get_gradient(current_pos), 0.0) ).xyz) ;
  gua_alpha = 1.0;
  gua_metalness  = 0.0;
  gua_roughness  = 0.9;
  gua_emissivity = 0.0; // pass through if unshaded

  gua_world_position = (gua_model_matrix * vec4(current_pos, 1.0)).xyz;


  @include "common/gua_write_gbuffer.glsl"

}