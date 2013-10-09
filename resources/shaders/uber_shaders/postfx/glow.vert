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

// input
layout(location=0) in vec3 gua_in_position;
layout(location=2) in vec2 gua_in_texcoord;

uniform float gua_glow_radius;
uniform float gua_texel_size;
uniform vec2  gua_blur_direction;

// output
out vec2 gua_quad_coords;
out vec2 blur_texcoords[14];

// body
void main() {
    gua_quad_coords = gua_in_texcoord;
    gl_Position = vec4(gua_in_position, 1.0);

    vec2 scale = gua_blur_direction * gua_texel_size * gua_glow_radius;

    blur_texcoords[ 0] = gua_quad_coords + vec2(-1.000) * scale;
    blur_texcoords[ 1] = gua_quad_coords + vec2(-0.857) * scale;
    blur_texcoords[ 2] = gua_quad_coords + vec2(-0.714) * scale;
    blur_texcoords[ 3] = gua_quad_coords + vec2(-0.571) * scale;
    blur_texcoords[ 4] = gua_quad_coords + vec2(-0.428) * scale;
    blur_texcoords[ 5] = gua_quad_coords + vec2(-0.285) * scale;
    blur_texcoords[ 6] = gua_quad_coords + vec2(-0.143) * scale;
    blur_texcoords[ 7] = gua_quad_coords + vec2( 0.143) * scale;
    blur_texcoords[ 8] = gua_quad_coords + vec2( 0.285) * scale;
    blur_texcoords[ 9] = gua_quad_coords + vec2( 0.428) * scale;
    blur_texcoords[10] = gua_quad_coords + vec2( 0.571) * scale;
    blur_texcoords[11] = gua_quad_coords + vec2( 0.714) * scale;
    blur_texcoords[12] = gua_quad_coords + vec2( 0.857) * scale;
    blur_texcoords[13] = gua_quad_coords + vec2( 1.000) * scale;
}
