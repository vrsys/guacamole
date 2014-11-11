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

uniform uvec2 gua_noise_tex;
uniform float gua_ssao_radius;
uniform float gua_ssao_intensity;
uniform float gua_ssao_falloff;

// output
layout(location=0) out vec4 gua_out_color;


float gua_ssao_aoFF(in vec3 ddiff,in vec3 cnorm, in float c1, in float c2, in vec2 texcoord) {
  float rd = length(ddiff);
  vec3 vv = ddiff / rd;
  return (1.0-clamp(dot(normalize(gua_get_normal(texcoord+vec2(c1,c2))),-vv),0.0,1.0)) *
        clamp(dot( cnorm,vv ),0.0,1.0) * (1.0 - 1.0/sqrt(1.0/(rd*rd*gua_ssao_falloff) + 1.0));
}

float gua_ssao_giFF(in vec3 ddiff,in vec3 cnorm, in float c1, in float c2, in vec2 texcoord, in sampler2D gnormals) {
  float rd = length(ddiff);
  vec3 vv = ddiff / rd;
  return 1.0*clamp(dot(normalize(gua_get_normal(texcoord+vec2(c1,c2))),-vv),0.0,1.0)*clamp(dot( cnorm,vv ),0.0,1.0)/(rd*rd+1.0);
}


void main() {
  vec2 texcoords = gua_get_quad_coords();

  vec3 n = normalize(gua_get_normal());
  vec3 p = gua_get_position();

  //randomization texture
  vec2 fres = vec2(1.0/(64.0*gua_texel_width)*5,1.0/(64.0*gua_texel_height)*5);
  vec3 random = texture2D(sampler2D(gua_noise_tex), texcoords.st*fres.xy).xyz;
  random = (random-vec3(0.5)) * gua_ssao_radius;

  //initialize variables
  float ao = 0.0;
  float incx = gua_texel_width * gua_ssao_radius;
  float incy = gua_texel_height * gua_ssao_radius;
  float pw = incx;
  float ph = incy;
  float cdepth = max(gua_get_depth(), 0.01);

  //3 rounds of 8 samples each.
  for(float i=0.0; i<4.0; ++i) {
    float npw = (pw+0.001*random.x)/cdepth;
    float nph = (ph+0.001*random.y)/cdepth;

    vec3 ddiff =  gua_get_position(texcoords+vec2(npw,nph))-p;
    vec3 ddiff2 = gua_get_position(texcoords+vec2(npw,-nph))-p;
    vec3 ddiff3 = gua_get_position(texcoords+vec2(-npw,nph))-p;
    vec3 ddiff4 = gua_get_position(texcoords+vec2(-npw,-nph))-p;
    vec3 ddiff5 = gua_get_position(texcoords+vec2(0,nph))-p;
    vec3 ddiff6 = gua_get_position(texcoords+vec2(0,-nph))-p;
    vec3 ddiff7 = gua_get_position(texcoords+vec2(npw,0))-p;
    vec3 ddiff8 = gua_get_position(texcoords+vec2(-npw,0))-p;

    ao+= gua_ssao_aoFF(ddiff, n, npw, nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff2, n, npw, -nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff3, n, -npw, nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff4, n, -npw, -nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff5, n, 0, nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff6, n, 0, -nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff7, n, npw, 0, texcoords);
    ao+= gua_ssao_aoFF(ddiff8, n, -npw, 0, texcoords);

    pw += incx;
    ph += incy;
  }

  ao/=32.0;

  gua_out_color = vec4(0.0, 0.0, 0.0, ao) * gua_ssao_intensity;
  // gua_out_color = vec4(ao, ao, ao, 1.0) * gua_ssao_intensity;
  // gua_out_color = gua_get_color() * max(0.0, 1.0 - ao*gua_ssao_intensity);
  // gua_out_color = gua_get_color() * max(0.0, 1.0 - ao);//*gua_ssao_intensity);
}
