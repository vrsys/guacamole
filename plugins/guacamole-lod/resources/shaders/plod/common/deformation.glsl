


vec3 sample_timestep_deformation(int current_integer_timestep) {
      int timestep_offset = current_integer_timestep * floats_per_attribute_timestep;

      vec3 deformation = vec3(0.0, 0.0, 0.0);

      for(int dim_idx = 0; dim_idx < 3; ++dim_idx) {

        int deformation_base_offset = floats_per_attribute_timestep * 2 * dim_idx + timestep_offset;
        deformation[dim_idx] =    fem_vert_w_0 * time_series_data[deformation_base_offset + fem_vert_id_0]
                                + fem_vert_w_1 * time_series_data[deformation_base_offset + fem_vert_id_1]
                                + fem_vert_w_2 * time_series_data[deformation_base_offset + fem_vert_id_2];
      }

      //gl_Position = vec4( deform_factor * deformation + in_position, 1.0);

     return deformation;

}

void deform_position(in out vec3 position) {
  if( ! (    (fem_vert_w_0 <= 0.0)  
          || (fem_vert_w_1 <= 0.0)  
          || (fem_vert_w_2 <= 0.0)
      )
     ) {
    
    int timestep_one = int(current_timestep);

    vec3 deformation_t0 = sample_timestep_deformation(0);

    vec3 final_deformation = deformation_t0;

    if(enable_linear_temporal_interpolation) {
      int timestep_two = timestep_one + 1;
      vec3 deformation_t1 = sample_timestep_deformation(1);

      float timestep_mixing_ratio = mod(current_timestep, 1.0);// - timestep_one;

      final_deformation = mix(deformation_t0, deformation_t1, timestep_mixing_ratio);


    }

    position += final_deformation * deform_factor;
  }
}