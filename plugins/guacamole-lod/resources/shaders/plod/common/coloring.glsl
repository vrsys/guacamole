

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
    factor = 0.3 + 0.0175 * (wavelength - 380.0);
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




