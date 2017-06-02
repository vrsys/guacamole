vec3 raycast_compositing(out vec4 out_composited_color) {
  vec3 ray_origin    = FragmentIn.pos_ms;

  vec3 ray_direction = normalize(FragmentIn.pos_ms - ms_eye_pos.xyz);



  vec3 ray_increment = ray_direction * step_size;

  vec3 current_pos = ray_origin;

  float max_intensity = 0.0;
  current_pos += ray_increment;

  float num_samples_taken = 0;

  vec4 color = vec4(0.0);

  while(is_inside_vol(current_pos) ) {

    // get sample
    float s   = get_mode_independent_sample(current_pos);

    float alpha = s < 0.5 ? 0.0 : s; 
    vec4  src = vec4(s,s,s,alpha);//texture(sampler2D(transfer_texture), vec2(s, 0.5));

    // compositing
    float omda_sa = (1.0 - color.a) * src.a;
    color.rgb += omda_sa * src.rgb;
    color.a   += omda_sa;

    current_pos +=  ray_increment;
    ++num_samples_taken;
  }

  out_composited_color = color.rgba;

  return current_pos;
};