/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

@include "shaders/common/header.glsl"

// varyings
in vec2 gua_quad_coords;

@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/gua_gbuffer_input.glsl"
@include "shaders/common/gua_resolve_pass_uniforms.glsl"

// output
layout(location=0) out vec3 gua_out_color;

vec3 linearToGamma(vec3 linearColor)
{
  return pow(linearColor, vec3(1/2.2));
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

vec3 Uncharted2Tonemap(vec3 x)
{
  return ((x*(A*x+C*B)+D*E)/(x*(A*x+B)+D*F))-E/F;
}

// includes pow(x,1/2.2)
// optimized formula by Jim Hejl and Richard Burgess-Dawson.
vec3 toneMapHejl(vec3 linearColor)
{
  linearColor *= gua_tone_mapping_exposure; // 16; // Hardcoded exposure adjustment
  //from comment section at http://filmicgames.com/archives/75
  //The 0.004 sets the value for the black point to give you a little more
  //contrast in the bottom end. The graph will look very close, you will see a
  //noticeable difference in the blacks in your images because the human eye has
  //more precision in the darker areas.
  vec3 x = max(vec3(0), linearColor - vec3(0.004));
  return (x * (6.2 * x + 0.5)) / ( x * (6.2 * x + vec3(1.7)) + vec3(0.06));
}

vec3 toneMapLinear(vec3 linearColor)
{
  linearColor *= gua_tone_mapping_exposure;
  return pow(linearColor, vec3(1.0/2.2));
}

void main()
{
  vec3 col = gua_get_color();
  switch (gua_tone_mapping_operator) {
    case 0:
      gua_out_color = toneMapLinear(col);
      break;
    case 1:
      gua_out_color = toneMapHejl(col);
      break;
    default:
      gua_out_color = toneMapLinear(col);
  }
}
