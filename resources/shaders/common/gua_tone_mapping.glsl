
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

vec3 toneMapReinhard(vec3 linearColor, float exposure)
{
  linearColor *= exposure; // 16.0; // Hardcoded exposure adjustment
  linearColor = linearColor/(1.0 + linearColor);
  return pow(linearColor, vec3(1.0/2.2));
}

// http://www.slideshare.net/ozlael/hable-john-uncharted2-hdr-lighting
// #define A 0.22f // shoulder strength
// #define B 0.30f // linear strength
// #define C 0.10 // Linear angle
// #define D 0.20 // Toe Strength
// #define E 0.01 // Toe Numerator
// E/F is Toe Angle
// F(x) = ((x*(A*x+C*B)+D*E)/(x*(A*x+B)+D*F))-E/F;
// FinalColor = F(LinearColor)/F(LinearWhite)

const float A = 0.15; // ShoulderStrength
const float B = 0.50; // LinearStrength
const float C = 0.10; // LinearAngle
const float D = 0.20; // ToeStrength
const float E = 0.02; // ToeNumerator
const float F = 0.30; // ToeDenominator
const float W = 11.2; // LinearWhite

vec3 Uncharted2Tonemap(vec3 linearColor)
{
  return ((linearColor*(A*linearColor+C*B)+D*E)/(linearColor*(A*linearColor+B)+D*F))-E/F;
}

vec3 toneMap(vec3 col)
{
  switch (@gua_tone_mapping_method@) {
    case 0:
      return toneMapLinear(col, gua_tone_mapping_exposure);
    case 1:
      return toneMapHejl(col, gua_tone_mapping_exposure);
    case 2:
      return toneMapReinhard(col, gua_tone_mapping_exposure);
    case 3:
      vec3 curr = Uncharted2Tonemap(gua_tone_mapping_exposure*col);
      vec3 whiteScale = 1.0f/Uncharted2Tonemap(vec3(W));
      vec3 color = curr*whiteScale;
      return pow(color, vec3(1.0/2.2));
  }
  return toneMapLinear(col, gua_tone_mapping_exposure);
}
