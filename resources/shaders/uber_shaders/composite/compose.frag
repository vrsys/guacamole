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

// input from gbuffer ----------------------------------------------------
uniform uvec2 gua_depth_gbuffer_in;
uniform uvec2 gua_color_gbuffer_in;
uniform uvec2 gua_normal_gbuffer_in;

// uniforms
@include "shaders/uber_shaders/common/get_sampler_casts.glsl"
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

// methods ---------------------------------------------------------------------

// global gua_* methods
vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

// write outputs ---------------------------------------------------------------------
layout(location=0) out vec3 gua_out_color;

// main ------------------------------------------------------------------------
void main() {
  // compose
  gua_out_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_get_quad_coords()).xyz +
                  texture2D(gua_get_float_sampler(gua_normal_gbuffer_in), gua_get_quad_coords()).xyz;
}

