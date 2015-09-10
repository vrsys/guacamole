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

@include "common/header.glsl"
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"
@include "gbuffer_warp_modes.glsl"
@include "hole_filling_modes.glsl"
@include "interpolation_modes.glsl"
@include "warp_grid_bits.glsl"

#if HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_1 || HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_2  || HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_3
  flat in uint is_rubber_band;
#endif

// -----------------------------------------------------------------------------
#if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD || WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_ADVANCED_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION
// -----------------------------------------------------------------------------

flat in uint cellsize;
in vec2 texcoords;

uniform uvec2 gua_warp_grid_tex;

// output
layout(location=0) out vec3 gua_out_color;

void main() {

  #if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD || WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION
    const bool is_surface = (texelFetch(usampler2D(gua_warp_grid_tex), ivec2(ivec2(texcoords*gua_resolution+vec2(0.001))/2), 0).x & 1) == 1;
  #else
    uint info = texelFetch(usampler2D(gua_warp_grid_tex), ivec2(ivec2(texcoords*gua_resolution+vec2(0.001))/2), 0).x;
    const bool is_surface = (info & ALL_CONTINUITY_BITS) == ALL_CONTINUITY_BITS;
  #endif

  #if INTERPOLATION_MODE == INTERPOLATION_MODE_NEAREST
    gua_out_color = texelFetch(sampler2D(gua_gbuffer_color), ivec2(texcoords*gua_resolution+vec2(0.001)), 0).rgb;
  #elif INTERPOLATION_MODE == INTERPOLATION_MODE_LINEAR
    gua_out_color = gua_get_color(texcoords);
  #else
    if (is_surface) {
      gua_out_color = gua_get_color(texcoords);
    } else {
      gua_out_color = texelFetch(sampler2D(gua_gbuffer_color), ivec2(texcoords*gua_resolution+vec2(0.001)), 0).rgb;
    }
  #endif

  #if @debug_cell_colors@ == 1
    float intensity = log2(cellsize) / 7.0;
    gua_out_color = mix(gua_out_color, vec3(0.4, 0.0, 0.0) * (1-intensity) + vec3(0.0, 0.4, 0.0) * intensity, 0.9);
  #endif

  #if @debug_interpolation_borders@ == 1
    if (!is_surface) {
      gua_out_color = mix(gua_out_color, vec3(0.0, 0.0, 0.8), 0.8);
    }
  #endif

  #if HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_1
    if (is_rubber_band == 1) {
      #if @debug_rubber_bands@ == 1
        gua_out_color = mix(gua_out_color, vec3(0.8, 0.0, 0.0), 0.5);
      #endif
    }
  #elif HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_2
    if (is_rubber_band == 1) {
      #if @debug_rubber_bands@ == 1
        gua_out_color = mix(gua_out_color, vec3(0.8, 0.0, 0.0), 0.5);
      #endif
      gl_FragDepth = 0.9999999;
    } else {
      gl_FragDepth = gl_FragCoord.z;
    }

  #endif
}


// -----------------------------------------------------------------------------
#else // all other modes -------------------------------------------------------
// -----------------------------------------------------------------------------

in vec3 color;
in vec3 normal;

// output
layout(location=0) out vec3 gua_out_color;

void main() {
  gua_out_color = color;
}

#endif
