///////////////////////////////////////////////////////////////////////////////
float depth_to_linear(float depth_log, float nearclip, float farclip) {
  return (2 * nearclip) / (farclip + nearclip - depth_log * (farclip - nearclip));
}

///////////////////////////////////////////////////////////////////////////////
float gaussian ( float x, float sigma, float mu ) {
  const float sqrt_two_pi = 2.506628274631;
  return ( 1.0 / (sigma * sqrt_two_pi) ) * exp ( -0.5 * ( ((x - mu) / sigma) * ((x - mu) / sigma) ));
}

///////////////////////////////////////////////////////////////////////////////
float compute_screen_space_shadow (int light_id, vec3 position) {

  // constant configuration
  const float sample_distance = 1.0; // allows to skip pixel -> default set off (1.0)
  const float gaussian_falloff = 0.5; // determines falloff towards outer radius of shadow

  LightSource light_params = gua_lights[light_id];

  if (!gua_screen_space_shadows_enable) {
    return 0.0; 
  } 

  // note: as screen space shadows are used as cheap alternative to shadow maps, this flag is not used
  //if (light_params.casts_shadow == false) {
  //  return 0.0;
  //}

  vec4 light_view        = gua_view_matrix * vec4(light_params.position_and_radius.xyz, 1.0);
  vec4 position_view     = gua_view_matrix * vec4(position, 1.0);
  vec4 light_to_pos_view = light_view - position_view;

  const vec3 z_plane   = vec3(0.0, 0.0, -1.0);
  const vec3 z_plane_p = vec3(0.0, 0.0, gua_clip_near);
  const vec3 L = normalize(light_to_pos_view.xyz);

  // transform light to unit cube and normalize to [0,0,0] - [1,1,1]
  vec4 light_screen = gua_projection_matrix * vec4(light_view.xyz, 1.0);
  light_screen /= light_screen.w;

  // transform pixel to unit cube and normalize to [0,0,0] - [1,1,1]
  vec4 pixel_screen = gua_projection_matrix * gua_view_matrix * vec4(position, 1.0);
  pixel_screen /= pixel_screen.w;
  vec3 pixel_normalized_coords = (pixel_screen.xyz + vec3(1.0)) / vec3(2.0); 

  vec2 light_direction = light_to_pos_view.xy;
  vec2 light_direction_normalized = normalize(light_direction) / vec2(gua_resolution.x, gua_resolution.y);

  float light_distance = length(light_to_pos_view.xyz);

  // sample to find depth
  //int max_sampling_distance_in_pixel = int(max(gua_resolution.x, gua_resolution.y));
  int max_sampling_distance_in_pixel = 500;
  for (int i = 1; i != gua_screen_space_shadows_max_radius_px; ++i) {

    vec2 sample_coords = pixel_normalized_coords.xy + sample_distance * i * light_direction_normalized;

    if ( sample_coords.x > 1.0 || sample_coords.x < 0.0 ||
         sample_coords.y > 1.0 || sample_coords.y < 0.0 ) 
    {
      return 0.0;
    }

    vec3 sample_pos = (gua_view_matrix * vec4(gua_get_position(sample_coords.xy), 1.0)).xyz;
    float sample_depth = sample_pos.z;

    float distance = length(sample_pos.xy - position_view.xy);
    if (distance > gua_screen_space_shadows_radius) {
      return 0.0;
    }

    float relative_sampling_distance = distance / light_distance;
    float light_depth = light_view.z;
    
    float shadow_threshold = position_view.z + relative_sampling_distance * (light_view.z - position_view.z);

    // allow 1% error 
    float error_threshold = 1e-2 * gua_screen_space_shadows_radius;
    if (sample_depth - error_threshold > shadow_threshold) {
      return min(gua_screen_space_shadows_intensity, gua_screen_space_shadows_intensity * gaussian(distance / gua_screen_space_shadows_radius, gaussian_falloff, 0));
    }
  }

  return 0.0;
}
