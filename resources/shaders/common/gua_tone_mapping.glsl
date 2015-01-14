
// includes pow(x,1/2.2)
// optimized formula by Jim Hejl and Richard Burgess-Dawson.
vec3 toneMapHejl(vec3 linearColor, float exposure)
{
  //from comment section at http://filmicgames.com/archives/75
  //The 0.004 sets the value for the black point to give you a little more
  //contrast in the bottom end. The graph will look very close, you will see a
  //noticeable difference in the blacks in your images because the human eye has
  //more precision in the darker areas.
  vec3 x = max(vec3(0), fma(linearColor, vec3(exposure), -vec3(0.004)));
  return (x * (6.2 * x + 0.5)) / ( x * (6.2 * x + vec3(1.7)) + vec3(0.06));
}

vec3 toneMapLinear(vec3 linearColor, float exposure)
{
  linearColor *= exposure; // 16.0; // Hardcoded exposure adjustment
  return pow(linearColor, vec3(1.0/2.2));
}

vec3 toneMap(vec3 col)
{
  switch (@tone_mapping_method@) {
    case 0:
      return toneMapLinear(col, @tone_mapping_exposure@);
    case 1:
      return toneMapHejl(col, @tone_mapping_exposure@);
  }
  return toneMapLinear(col, @tone_mapping_exposure@);
}
