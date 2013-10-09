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
uniform float gua_luminance;
uniform float gua_hdr_key;

// write outputs
layout(location=0) out vec3 gua_out_color;

// print global gua_* methods
vec2 gua_get_quad_coords() {
    return gua_quad_coords;
}

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

//////////////////////////////// HDR ///////////////////////////////////

void main() {
    gua_out_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_quad_coords).xyz;

    const float min_luminance = 0.01;
    const float max_luminance = 10.0;

    float luminance = clamp(gua_luminance, min_luminance, max_luminance);
    float rel_luminance = gua_hdr_key * dot(vec3(0.2126, 0.7152, 0.0722), gua_out_color) / luminance;

    gua_out_color *= rel_luminance / (1.0 + rel_luminance);
}
