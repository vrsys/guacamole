@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in Data {
  vec3 pos_ms;
} FragmentIn;

layout(binding=0) uniform sampler3D volume_texture;

uniform vec4 ms_eye_pos;

uniform float gua_texel_width;
uniform float gua_texel_height;


layout(location = 0) out vec4 out_color;

//returns model space t (where t=0.0 old starting point & t= 1.0)
//front and back orientation need to be distinguished


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

void main() {

  vec3 ray_origin    = ms_eye_pos.xyz;
  vec3 ray_direction = normalize(FragmentIn.pos_ms - ms_eye_pos.xyz);


  vec2 t_min_max = intersect_ray_with_unit_box(ray_origin, ray_direction);

  vec3 ray_increment = ray_direction * step_size;

  vec3 current_pos = vec3(0.0, 0.0, 0.0);

  if(t_min_max[0] < 0) {
	current_pos = ms_eye_pos.xyz;
  } else {
	current_pos = ray_origin + t_min_max[0] * ray_direction;
  }
  current_pos += ray_increment;

  vec3 cam_to_box_end_vec =  FragmentIn.pos_ms - current_pos;

  t_min_max[0] = 0.0;
  t_min_max[1] = 1.0;
/*
  //plane intersection
  compute_ray_plane_intersection(vec3(0.5, 0.5, 0.5), vec3(0.0, 0.0, 0.5),
  	                             current_pos, cam_to_box_end_vec,
  	                             t_min_max[0], t_min_max[1]);
*/
  //current_pos = current_pos + t_min_max[0] * cam_to_box_end_vec;

 // current_pos += ray_increment;

  float max_intensity = 0.0;

  int num_samples = 0;

  while(is_inside_vol(current_pos) ) {

  	max_intensity = max(max_intensity, texture(volume_texture, current_pos ).r);
  	current_pos += ray_increment;
  	++num_samples;
  }

  float alpha = 0.0;
  if(num_samples != 0) {
  	alpha = 1.0;
  }

    //float density = texture(volume_texture, current_pos ).r;
    out_color = vec4(max_intensity,max_intensity,max_intensity, alpha);	
 	


  //outColor = current_pos;
  //outColor = vec3(VertexIn.pos_ms);
}