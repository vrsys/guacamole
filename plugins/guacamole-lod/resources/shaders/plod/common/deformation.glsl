void deform_position(in out vec3 position) {
  if( ! (    (fem_vert_w_0 <= 0.0)  
          || (fem_vert_w_1 <= 0.0)  
          || (fem_vert_w_2 <= 0.0)
      )
     ) {
    
    int timestep_offset = int(current_timestep) * floats_per_attribute_timestep;

    vec3 deformation = vec3(0.0, 0.0, 0.0);

    for(int dim_idx = 0; dim_idx < 3; ++dim_idx) {

      int deformation_base_offset = attribute_offset * dim_idx + timestep_offset;
      deformation[dim_idx] =    fem_vert_w_0 * time_series_data[deformation_base_offset + fem_vert_id_0]
                              + fem_vert_w_1 * time_series_data[deformation_base_offset + fem_vert_id_1]
                              + fem_vert_w_2 * time_series_data[deformation_base_offset + fem_vert_id_2];
    }

    //gl_Position = vec4( deform_factor * deformation + in_position, 1.0);

    position += deformation * deform_factor;

    //VertexOut.pass_point_color = mix(data_value_to_rainbow(mixed_value, min_ssbo_value, max_ssbo_value), VertexOut.pass_point_color, 0.7);
  }
}