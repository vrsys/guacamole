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

// uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

uniform bool gua_enable_fxaa;
uniform bool gua_enable_vignette;

uniform float gua_vignette_softness;
uniform float gua_vignette_coverage;
uniform vec3 gua_vignette_color;

// write outputs
layout(location=0) out vec3 gua_out_color;

// print global gua_* methods
vec2 gua_get_quad_coords() {
    return gua_quad_coords;
}

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

///////////////////////////// FXAA /////////////////////////////////////

void gua_apply_fxaa() {
    // The parameters are hardcoded for now, but could be
    // made into uniforms to control from  the program.
    float FXAA_SPAN_MAX = 8.0;
    float FXAA_REDUCE_MUL = 1.0/8.0;
    float FXAA_REDUCE_MIN = (1.0/128.0);

    vec2 texcoordOffset = vec2(gua_texel_width, gua_texel_height);
    sampler2D gdiffuse = gua_get_float_sampler(gua_color_gbuffer_in);
    vec2 texcoords = gua_get_quad_coords();

    vec3 rgbNW = texture2D(gdiffuse, texcoords.xy + (vec2(-1.0, -1.0) * texcoordOffset)).xyz;
    vec3 rgbNE = texture2D(gdiffuse, texcoords.xy + (vec2(+1.0, -1.0) * texcoordOffset)).xyz;
    vec3 rgbSW = texture2D(gdiffuse, texcoords.xy + (vec2(-1.0, +1.0) * texcoordOffset)).xyz;
    vec3 rgbSE = texture2D(gdiffuse, texcoords.xy + (vec2(+1.0, +1.0) * texcoordOffset)).xyz;
    vec3 rgbM = texture2D(gdiffuse, texcoords.xy).xyz;

    vec3 luma = vec3(0.299, 0.587, 0.114);
    float lumaNW = dot(rgbNW, luma);
    float lumaNE = dot(rgbNE, luma);
    float lumaSW = dot(rgbSW, luma);
    float lumaSE = dot(rgbSE, luma);
    float lumaM = dot( rgbM, luma);

    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

    vec2 dir;
    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    dir.y = ((lumaNW + lumaSW) - (lumaNE + lumaSE));

    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * FXAA_REDUCE_MUL), FXAA_REDUCE_MIN);

    float rcpDirMin = 1.0/(min(abs(dir.x), abs(dir.y)) + dirReduce);

    dir = min(vec2(FXAA_SPAN_MAX, FXAA_SPAN_MAX),
              max(vec2(-FXAA_SPAN_MAX, -FXAA_SPAN_MAX), dir * rcpDirMin)) * texcoordOffset;

    vec3 rgbA = (1.0/2.0) * (
                texture2D(gdiffuse, texcoords.xy + dir * (1.0/3.0 - 0.5)).xyz +
                texture2D(gdiffuse, texcoords.xy + dir * (2.0/3.0 - 0.5)).xyz);
    vec3 rgbB = rgbA * (1.0/2.0) + (1.0/4.0) * (
                texture2D(gdiffuse, texcoords.xy + dir * (0.0/3.0 - 0.5)).xyz +
                texture2D(gdiffuse, texcoords.xy + dir * (3.0/3.0 - 0.5)).xyz);
    float lumaB = dot(rgbB, luma);

    if((lumaB < lumaMin) || (lumaB > lumaMax)){
        gua_out_color = rgbA;
    } else {
        gua_out_color = rgbB;
    }
}

///////////////////////////// VIGNETTE /////////////////////////////////

void apply_vignette() {

    float hardness = gua_vignette_softness;
    float offset = 0.5 - gua_vignette_coverage*0.5;

    float dist = length(gua_get_quad_coords() - vec2(0.5));
    float fac = (dist - offset)/hardness;
    fac = 1.0 - pow(clamp(fac, 0.0, 1.0), 2);

    gua_out_color = gua_out_color * fac + (1.0 - fac) * gua_vignette_color;
}

/////////////////////////////// main ///////////////////////////////////

void main() {
    gua_out_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_quad_coords).xyz;

    if (gua_enable_fxaa)        gua_apply_fxaa();
    if (gua_enable_vignette)    apply_vignette();
}
