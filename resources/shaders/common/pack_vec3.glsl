// migrate 32bit float to 8bit uchar
unsigned char float_to_uchar(float value)
{
  value = (value + 1.0f) * 0.5;
  return (unsigned char)(value*255.0f);
}

// 
float uchar3_to_float(unsigned char x, unsigned char y, unsigned char z)
{
  unsigned int packedColor = unsigned int((unsigned int(x) << 16) | (unsigned int(y) << 8) | unsigned int(z));
  float packedFloat = (float)(((double)packedColor) / ((double)(1 << 24)));
  return packedFloat;
}

// pack normalized vec3 to single float
float pack_vec3(vec3 input)
{
  return uchar3_to_float(float_to_uchar(input.r), float_to_uchar(input.g), float_to_uchar(input.b));
}

vec3 unpack_multiplicator = vec3(1.0, 256.0, 65536.0);
// unpack 32bit packed vec3
vec3 unpack_vec3(float src)
{
  vec3 rgb = fract(src * unpack_multiplicator);
  //float r = fract(src);
  //float g = fract(src * 256.0);
  //float b = fract(src * 65536.0);

  //Unpack to the -1..1 range
  rgb *=  2.0;
  rgb -= 1.0;
  //r = (r * 2.0) - 1.0;
  //g = (g * 2.0) - 1.0;
  //b = (b * 2.0) - 1.0;

  return vec3(rgb.r, rgb.g, rgb.b);
}
