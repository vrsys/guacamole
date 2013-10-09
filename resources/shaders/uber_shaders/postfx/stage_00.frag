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

// input from vertex shader
in vec2 gua_quad_coords;

// input from gbuffer
uniform uvec2 gua_depth_gbuffer_in;
uniform uvec2 gua_color_gbuffer_in;
uniform uvec2 gua_normal_gbuffer_in;

// write uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

uniform float gua_ssao_intensity;
uniform float gua_ssao_radius;
uniform float gua_ssao_falloff;
uniform uvec2 gua_noise_tex;

uniform bool gua_enable_ssao;
uniform bool gua_enable_godrays;

uniform bool gua_enable_fog;
uniform float gua_fog_start;
uniform float gua_fog_end;
uniform bool gua_fog_is_color;
uniform vec3 gua_fog_color;
uniform uvec2 gua_fog_texture;

uniform uvec2 gua_god_rays_texture;

// write outputs
layout(location=0) out vec3 gua_out_color;

// print global gua_* methods
vec2 gua_get_quad_coords() {
  return gua_quad_coords;
}

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"
@include "shaders/uber_shaders/common/get_depth.glsl"
@include "shaders/uber_shaders/common/get_position.glsl"

/////////////////////////// GOD RAYS ///////////////////////////////////

void gua_apply_god_rays() {
  gua_out_color = gua_out_color + texture2D(gua_get_float_sampler(gua_god_rays_texture), gua_quad_coords).rgb;
}

/////////////////////////////// FOG ////////////////////////////////////

float my_atan2(float a, float b) {
  return 2.0 * atan(a/(sqrt(b*b + a*a) + b));
}

vec3 gua_get_skymap() {
  vec3 pos = gua_get_position();
  vec3 view = normalize(pos - gua_camera_position);

  const float pi = 3.14159265359;
  float x = 0.5 + 0.5*my_atan2(view.x, -view.z)/pi;
  float y = 1.0 - acos(view.y)/pi;

  vec2 texcoord = vec2(x, y);

  return texture2D(gua_get_float_sampler(gua_fog_texture), texcoord).xyz;
}

void gua_apply_fog() {
  if (gua_get_depth() < 1.0) {
    float dist = length(gua_camera_position - gua_get_position());
    float fog_factor = clamp((dist - gua_fog_start)/(gua_fog_end - gua_fog_start), 0.0, 1.0);

    if (gua_fog_is_color) gua_out_color = mix(gua_out_color, gua_fog_color, fog_factor);
    else                  gua_out_color = mix(gua_out_color, gua_get_skymap(), fog_factor);
  }
}

///////////////////////////// SSAO /////////////////////////////////////

vec3 gua_ssao_read_normal(in vec2 coord, in sampler2D gnormals) {
  return normalize(texture2D(gnormals, coord).xyz);
}

vec3 gua_ssao_read_position(in vec2 coord) {
  return gua_get_position(coord);
}

float gua_ssao_aoFF(in vec3 ddiff,in vec3 cnorm, in float c1, in float c2, in vec2 texcoord, in sampler2D gnormals) {
  float rd = length(ddiff);
  vec3 vv = ddiff / rd;
  return (1.0-clamp(dot(gua_ssao_read_normal(texcoord+vec2(c1,c2), gnormals),-vv),0.0,1.0)) *
        clamp(dot( cnorm,vv ),0.0,1.0) * (1.0 - 1.0/sqrt(1.0/(rd*rd*gua_ssao_falloff) + 1.0));
}

float gua_ssao_giFF(in vec3 ddiff,in vec3 cnorm, in float c1, in float c2, in vec2 texcoord, in sampler2D gnormals) {
  float rd = length(ddiff);
  vec3 vv = ddiff / rd;
  return 1.0*clamp(dot(gua_ssao_read_normal(texcoord+vec2(c1,c2), gnormals),-vv),0.0,1.0)*clamp(dot( cnorm,vv ),0.0,1.0)/(rd*rd+1.0);
}

void gua_apply_ssao() {
  vec2 texcoords = gua_get_quad_coords();

  sampler2D gdiffuse = gua_get_float_sampler(gua_color_gbuffer_in);
  sampler2D gnormals = gua_get_float_sampler(gua_normal_gbuffer_in);

  vec3 n = gua_ssao_read_normal(texcoords, gnormals);
  vec3 p = gua_ssao_read_position(texcoords);
  vec3 col = texture2D(gdiffuse, texcoords).rgb;

  //randomization texture
  vec2 fres = vec2(1.0/(64.0*gua_texel_width)*5,1.0/(64.0*gua_texel_height)*5);
  vec3 random = texture2D(gua_get_float_sampler(gua_noise_tex), texcoords.st*fres.xy).xyz;
  random = (random-vec3(0.5))*gua_ssao_radius;

  //initialize variables
  float ao = 0.0;
  float incx = gua_texel_width*gua_ssao_radius;
  float incy = gua_texel_height*gua_ssao_radius;
  float pw = incx;
  float ph = incy;
  float cdepth = max(gua_get_depth(), 0.01);

  //3 rounds of 8 samples each.
  for(float i=0.0; i<4.0; ++i) {
    float npw = (pw+0.001*random.x)/cdepth;
    float nph = (ph+0.001*random.y)/cdepth;

    vec3 ddiff = gua_ssao_read_position(texcoords+vec2(npw,nph))-p;
    vec3 ddiff2 = gua_ssao_read_position(texcoords+vec2(npw,-nph))-p;
    vec3 ddiff3 = gua_ssao_read_position(texcoords+vec2(-npw,nph))-p;
    vec3 ddiff4 = gua_ssao_read_position(texcoords+vec2(-npw,-nph))-p;
    vec3 ddiff5 = gua_ssao_read_position(texcoords+vec2(0,nph))-p;
    vec3 ddiff6 = gua_ssao_read_position(texcoords+vec2(0,-nph))-p;
    vec3 ddiff7 = gua_ssao_read_position(texcoords+vec2(npw,0))-p;
    vec3 ddiff8 = gua_ssao_read_position(texcoords+vec2(-npw,0))-p;

    ao+= gua_ssao_aoFF(ddiff,n,npw,nph,texcoords,gnormals);
    ao+= gua_ssao_aoFF(ddiff2,n,npw,-nph,texcoords,gnormals);
    ao+= gua_ssao_aoFF(ddiff3,n,-npw,nph,texcoords,gnormals);
    ao+= gua_ssao_aoFF(ddiff4,n,-npw,-nph,texcoords,gnormals);
    ao+= gua_ssao_aoFF(ddiff5,n,0,nph,texcoords,gnormals);
    ao+= gua_ssao_aoFF(ddiff6,n,0,-nph,texcoords,gnormals);
    ao+= gua_ssao_aoFF(ddiff7,n,npw,0,texcoords,gnormals);
    ao+= gua_ssao_aoFF(ddiff8,n,-npw,0,texcoords,gnormals);

    pw += incx;
    ph += incy;
  }

  ao/=32.0;

  gua_out_color = gua_out_color * max(0.0, 1.0 - ao*gua_ssao_intensity);
}

/////////////////////////////// main ///////////////////////////////////

void main() {
  gua_out_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_quad_coords).xyz;

  if (gua_enable_ssao) gua_apply_ssao();
  if (gua_enable_fog) gua_apply_fog();
  if (gua_enable_godrays) gua_apply_god_rays();
}

