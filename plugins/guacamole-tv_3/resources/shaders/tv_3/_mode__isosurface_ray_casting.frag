vec3 raycast_isosurface() {
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