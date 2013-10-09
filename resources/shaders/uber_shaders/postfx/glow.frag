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

subroutine vec3 GetColorType();
subroutine uniform GetColorType get_color;

uniform uvec2 gua_glow_texture;
uniform float gua_glow_threshold;

in vec2 gua_quad_coords;
in vec2 blur_texcoords[14];

// outputs
layout(location=0) out vec3 gua_out_color;

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

subroutine( GetColorType )
vec3 get_color_threshold() {
    vec3 color = texture2D( gua_get_float_sampler(gua_glow_texture), gua_quad_coords).rgb;
    color = color + 1.0 - gua_glow_threshold;
    color.r = pow(color.r, 10);
    color.g = pow(color.g, 10);
    color.b = pow(color.b, 10);
    return min(vec3(1.0), color);
}

subroutine( GetColorType )
vec3 get_color_smooth() {

    vec3 result = vec3(0);
    sampler2D tex = gua_get_float_sampler(gua_glow_texture);

    result += texture2D(tex, blur_texcoords[ 0]).rgb*0.0044299121055113265;
    result += texture2D(tex, blur_texcoords[ 1]).rgb*0.00895781211794;
    result += texture2D(tex, blur_texcoords[ 2]).rgb*0.0215963866053;
    result += texture2D(tex, blur_texcoords[ 3]).rgb*0.0443683338718;
    result += texture2D(tex, blur_texcoords[ 4]).rgb*0.0776744219933;
    result += texture2D(tex, blur_texcoords[ 5]).rgb*0.115876621105;
    result += texture2D(tex, blur_texcoords[ 6]).rgb*0.147308056121;
    result += texture2D(tex, gua_quad_coords   ).rgb*0.159576912161;
    result += texture2D(tex, blur_texcoords[ 7]).rgb*0.147308056121;
    result += texture2D(tex, blur_texcoords[ 8]).rgb*0.115876621105;
    result += texture2D(tex, blur_texcoords[ 9]).rgb*0.0776744219933;
    result += texture2D(tex, blur_texcoords[10]).rgb*0.0443683338718;
    result += texture2D(tex, blur_texcoords[11]).rgb*0.0215963866053;
    result += texture2D(tex, blur_texcoords[12]).rgb*0.00895781211794;
    result += texture2D(tex, blur_texcoords[13]).rgb*0.0044299121055113265;

    return result;
}

void main() {
    gua_out_color = get_color();
}
