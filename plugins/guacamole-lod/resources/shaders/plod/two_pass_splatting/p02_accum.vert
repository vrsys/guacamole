@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

// input attributes
layout (location = 0) in vec3  in_position;
layout (location = 1) in float in_r;
layout (location = 2) in float in_g;
layout (location = 3) in float in_b;
layout (location = 4) in float empty;
layout (location = 5) in float in_radius;
layout (location = 6) in vec3 in_normal;

layout(location = 7) in int fem_vert_id_0;
layout(location = 8) in int fem_vert_id_1;
layout(location = 9) in int fem_vert_id_2;
layout(location = 10) in float fem_vert_w_0;
layout(location = 11) in float fem_vert_w_1;
layout(location = 12) in float fem_vert_w_2;

layout (std430, binding = 20) coherent readonly buffer time_series_data_ssbo {
  float[] time_series_data;
};


uniform uint gua_material_id;
uniform float radius_scaling;
uniform float max_surfel_radius;

uniform int floats_per_attribute_timestep;

uniform int attribute_offset;

uniform int current_timestep;

uniform float min_ssbo_value;
uniform float max_ssbo_value;

out VertexData {
  //output to geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;

  vec3 pass_point_color;
  vec3 pass_normal;
} VertexOut;



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
  float Blue;
  float factor;
  float Green;
  float Red;
  if(380.0 <= wavelength && wavelength <= 440.0){
    Red   = -(wavelength - 440.0) / (440.0 - 380.0);
    Green = 0.0;
    Blue  = 1.0;
  }
  else if(440.0 < wavelength && wavelength <= 490.0){
    Red   = 0.0;
    Green = (wavelength - 440.0) / (490.0 - 440.0);
    Blue  = 1.0;
  }
  else if(490.0 < wavelength && wavelength <= 510.0){
    Red   = 0.0;
    Green = 1.0;
    Blue  = -(wavelength - 510.0) / (510.0 - 490.0);
  }
  else if(510.0 < wavelength && wavelength <= 580.0){
    Red   = (wavelength - 510.0) / (580.0 - 510.0);
    Green = 1.0;
    Blue  = 0.0;
  }
  else if(580.0 < wavelength && wavelength <= 645.0){   
    Red   = 1.0;
    Green = -(wavelength - 645.0) / (645.0 - 580.0);
    Blue  = 0.0;
  }
  else if(645.0 < wavelength && wavelength <= 780.0){
    Red   = 1.0;
    Green = 0.0;
    Blue  = 0.0;
  }
  else{
    Red   = 0.0;
    Green = 0.0;
    Blue  = 0.0;
  }
  
  
  if(380.0 <= wavelength && wavelength <= 420.0){
    factor = 0.3 + 0.7*(wavelength - 380.0) / (420.0 - 380.0);
  }
  else if(420.0 < wavelength && wavelength <= 701.0){
    factor = 1.0;
  }
  else if(701.0 < wavelength && wavelength <= 780.0){
    factor = 0.3 + 0.7*(780.0 - wavelength) / (780.0 - 701.0);
  }
  else{
    factor = 0.0;
  }
  float R = adjust(Red,   factor);
  float G = adjust(Green, factor);
  float B = adjust(Blue,  factor);
  return vec3(R/255.0,G/255.0,B/255.0);
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










void main() {
  @include "../common_LOD/PLOD_vertex_pass_through.glsl"

  VertexOut.pass_point_color = vec3(in_r, in_g, in_b);


  //if(fem_vert_w_0 != 0.0) {
  if( ! (    (fem_vert_w_0 <= 0.0)  
          || (fem_vert_w_1 <= 0.0)  
          || (fem_vert_w_2 <= 0.0)
      )
     ) {
    
    int timestep_offset = current_timestep * floats_per_attribute_timestep;

    //int attribute_offset = timestep_offset / 10;

    //int time_attributes_per_offset = floats_per_timestep; 

    //int access_id = gl_VertexID;

    float mixed_value =   fem_vert_w_0 * time_series_data[attribute_offset * 3 + timestep_offset + fem_vert_id_0]
                        + fem_vert_w_1 * time_series_data[attribute_offset * 3 + timestep_offset + fem_vert_id_1]
                        + fem_vert_w_2 * time_series_data[attribute_offset * 3 + timestep_offset + fem_vert_id_2];






    vec3 deformation = vec3(0.0, 0.0, 0.0);

    for(int dim_idx = 0; dim_idx < 3; ++dim_idx) {
      deformation[dim_idx] =    fem_vert_w_0 * time_series_data[attribute_offset * dim_idx + timestep_offset + fem_vert_id_0]
                              + fem_vert_w_1 * time_series_data[attribute_offset * dim_idx + timestep_offset + fem_vert_id_1]
                              + fem_vert_w_2 * time_series_data[attribute_offset * dim_idx + timestep_offset + fem_vert_id_2];
    }



    vec4 deform = vec4(deformation, 0.0);

    //mat4 transform = mat4(0.867211, 0.497952, 0.00774202, -276.714, 0.498012, -0.867097, -0.0136602, 2758.97, -9.0027e-05, 0.0157017, -0.999951, 202.809, 0, 0, 0, 1);

    //deform = transform * deform;


    gl_Position = vec4( 1000 * deform.xyz + in_position, 1.0);

    //VertexOut.pass_point_color = vec3(0.01 * (mixed_value), 0.0, 0.0);
    //VertexOut.pass_point_color = vec3(1.0, 0.0, 0.0);
    
    //float normalized_value = (mixed_value) / (max_ssbo_value );
    //VertexOut.pass_point_color = vec3(normalized_value, 0.0, 0.0);


    VertexOut.pass_point_color = mix(data_value_to_rainbow(mixed_value, min_ssbo_value, max_ssbo_value), VertexOut.pass_point_color, 0.7);

    //if(mixed_value > 0.0 && mixed_value < max_ssbo_value) {
    //  VertexOut.pass_point_color = vec3(0.0, 1.0, 1.0);
    //}

    //if(time_series_data[0] > 0.0 && time_series_data[0] < 6.0) {
    //  VertexOut.pass_point_color = vec3(1.0, 0.0, 1.0);   
    //}

    //if(mixed_value > max_ssbo_value || mixed_value < 0.0) {
    //  VertexOut.pass_point_color = vec3(1.0, 0.0, 1.0);   
    //}

    //VertexOut.pass_point_color = vec3(float(fem_vert_id_0) / 105000.0, 0.0, 0.0);

    //VertexOut.pass_point_color = vec3(time_series_data[0], 0.0, 0.0);

  } else {

  }

  /*
  if(fem_vert_id_0 > 30000 && fem_vert_id_1 > 3 && fem_vert_id_2 > 3) {
    VertexOut.pass_point_color = vec3(0.0, 1.0, 0.0);
  }
  */

  VertexOut.pass_normal = in_normal;


}