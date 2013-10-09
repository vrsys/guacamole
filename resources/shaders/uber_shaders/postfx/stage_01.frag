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
uniform uvec2 gua_color_gbuffer_in;

// write uniforms
uniform uvec2 gua_glow_texture;
uniform float gua_glow_intensity;

// write outputs
layout(location=0) out vec3 gua_out_color;

// print global gua_* methods
vec2 gua_get_quad_coords() {
    return gua_quad_coords;
}

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

/////////////////////////////// GLOW ///////////////////////////////////

void main() {
    gua_out_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_quad_coords).xyz;
    gua_out_color = gua_out_color + texture2D(gua_get_float_sampler(gua_glow_texture), gua_quad_coords).xyz * gua_glow_intensity;
}
