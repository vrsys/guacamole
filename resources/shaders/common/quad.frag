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

uniform uvec2 gua_in_texture;

in vec2 gua_quad_coords;
in vec3 gua_normal;

@include "shaders/common/gua_fragment_shader_output.glsl"

void main() {
    //vec3 gua_color = texture2D(sampler2D(gua_in_texture), gua_quad_coords).rgb;
    vec4 gua_color = texture2DArray(sampler2DArray(gua_in_texture), vec3((gua_varying_quad_coords - 0.5)*flip + 0.5, gl_Layer ) );
    // vec3 gua_color = vec3(gua_quad_coords, 0);
    float gua_emissivity = 1.0;
    float gua_roughness = 0.0;
    float gua_metalness = 0.0;

    @include "shaders/common/gua_write_gbuffer.glsl"
}
