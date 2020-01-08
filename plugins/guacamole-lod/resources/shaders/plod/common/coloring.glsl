

uniform float min_active_attribute_value;
uniform float max_active_attribute_value;

// --------------------------
const float GAMMA        = 0.80;
const float INTENSITY_MAX = 255.0;
  
float round(float d){
  return floor(d + 0.5);
}
  
float adjust(in float color, in float factor){
  if (color == 0.0){
    return 0.0;
  }
  else{
    float res = round(INTENSITY_MAX * pow(color * factor, GAMMA));
    return min(255.0, max(0.0, res));
  }
}

vec3 wavelength_to_RGB(in float wavelength){

  float factor = 0.0;

  vec3 RGB = vec3(0.0);

  if(380.0 <= wavelength && wavelength <= 440.0){
    RGB.r  = -0.016666666666666666 * (wavelength - 440.0);
    RGB.b  = 1.0;
  }
  else if(440.0 < wavelength && wavelength <= 490.0){

    RGB.g = 0.02 * (wavelength - 440.0);
    RGB.b = 1.0;
  }
  else if(490.0 < wavelength && wavelength <= 510.0){
    RGB.g = 1.0;
    RGB.b = -(wavelength - 510.0) / (510.0 - 490.0);
  }
  else if(510.0 < wavelength && wavelength <= 580.0){
    RGB.r = 0.014285714285714285 * (wavelength - 510.0);
    RGB.g = 1.0;
  }
  else if(580.0 < wavelength && wavelength <= 645.0){   
    RGB.r = 1.0;
    RGB.g = -0.015384615384615385 * (wavelength - 645.0);
  }
  else if(645.0 < wavelength && wavelength <= 780.0){
    RGB.r = 1.0;
  }

  
  if(380.0 <= wavelength && wavelength <= 420.0){
    factor = 0.0175 * (wavelength - 380.0) + 0.3;
  }
  else if(420.0 < wavelength && wavelength <= 701.0){
    factor = 1.0;
  }
  else if(701.0 < wavelength && wavelength <= 780.0){
    factor = 0.008860759493670885 * (780.0 - wavelength) + 0.3;
  }

  RGB = vec3(adjust(RGB.r, factor), 
             adjust(RGB.g, factor),
             adjust(RGB.b, factor)
            );

  return 0.00392156862745098 * vec3(RGB);
}
  
float get_wavelength_from_data_point(float value, float min_value, float max_value){
  float min_visible_wavelength = 380.0;//350.0;
  float max_visible_wavelength = 780.0;//650.0;
  //Convert data value in the range of MinValues..MaxValues to the 
  //range 350..780
  return (value - min_value) / (max_value-min_value) * (max_visible_wavelength - min_visible_wavelength) + min_visible_wavelength;
} 

vec3 data_value_to_rainbow(float value, float min_value, float max_value) {
  float wavelength = get_wavelength_from_data_point(value, min_value, max_value);
  return wavelength_to_RGB(wavelength);   
}

vec3 sample_attribute_color( ) {
  vec3 vertex_weights = vec3(fem_vert_w_0, fem_vert_w_1, fem_vert_w_2);
  ivec3 vertex_ids    = ivec3(fem_vert_id_0, fem_vert_id_1, fem_vert_id_2);

  int timestep_0 = int(0);

  int timestep_offset_0 = timestep_0 * floats_per_attribute_timestep;

  //attribute to visualize will always be in slot 4 from now on
  uint precomputed_attribute_offset_0 = floats_per_attribute_timestep * 2 * 3 + timestep_offset_0;
  vec3 looked_up_vertex_values_0 = vec3(time_series_data[precomputed_attribute_offset_0 + vertex_ids.x],
                                        time_series_data[precomputed_attribute_offset_0 + vertex_ids.y],
                                        time_series_data[precomputed_attribute_offset_0 + vertex_ids.z]);

  float mixed_value_t0 = dot(vertex_weights, looked_up_vertex_values_0);
  float final_mixed_value = mixed_value_t0;


  if(enable_linear_temporal_interpolation) {
    int timestep_1 = timestep_0 + 1;

    int timestep_offset_1 = timestep_offset_0 + floats_per_attribute_timestep;

    //attribute to visualize will always be in slot 4 from now on
    uint precomputed_attribute_offset_1 = floats_per_attribute_timestep * 2 * 3 + timestep_offset_1;
    vec3 looked_up_vertex_values_1 = vec3(time_series_data[precomputed_attribute_offset_1 + vertex_ids.x],
                                          time_series_data[precomputed_attribute_offset_1 + vertex_ids.y],
                                          time_series_data[precomputed_attribute_offset_1 + vertex_ids.z]);

    float mixed_value_t1 = dot(vertex_weights, looked_up_vertex_values_1);
    float timestep_mixing_ratio = mod(current_timestep, 1.0);

    final_mixed_value = mix(mixed_value_t0, mixed_value_t1, timestep_mixing_ratio);
  }
  
  return data_value_to_rainbow(final_mixed_value, min_active_attribute_value, max_active_attribute_value);
}