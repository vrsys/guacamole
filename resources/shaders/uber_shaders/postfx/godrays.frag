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

subroutine float GetColorType(vec2 texcoords);
subroutine uniform GetColorType get_color;

uniform uvec2 gua_ray_texture;
uniform float gua_filter_length;
uniform float gua_aspect_ratio;
uniform vec3 gua_light_color;

in vec3 gua_light_position_screen_space;
in vec2 gua_quad_coords;

// outputs
layout(location=0) out vec3 gua_out_color;


@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

subroutine( GetColorType )
float get_color_clamped(vec2 texcoords) {
    float depth = texture2D( gua_get_float_sampler(gua_ray_texture), texcoords).r * 2 -1;
    float intensity = depth >= gua_light_position_screen_space.z ? 1.0 : 0.0;
    intensity *= max(0.0, 1.0-length((gua_quad_coords - gua_light_position_screen_space.xy * 0.5 - 0.5)/vec2(1.0, gua_aspect_ratio)));
    return pow(intensity, 15.0);
}

subroutine( GetColorType )
float get_color_smooth(vec2 texcoords) {
    return texture2D( gua_get_float_sampler(gua_ray_texture), texcoords).r;
}

void main() {
    const float samples = 6.0;

    vec2 light_position = gua_light_position_screen_space.xy * 0.5 + 0.5;
    vec2 delta = light_position - gua_quad_coords;
    vec2 stepv =  delta / (samples * gua_filter_length);
    vec2 texcoords = gua_quad_coords;

    float col = 0.0;
    for (float i = 0.0; i < samples; i += 1.0) {
        col += get_color(texcoords);
        texcoords += stepv;
    }

    gua_out_color = col / samples * gua_light_color;
}
