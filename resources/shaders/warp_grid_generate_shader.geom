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
@include "gbuffer_warp_modes.glsl"
@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/warp_grid_bits.glsl"

layout(points) in;
layout(points, max_vertices = 4) out;

in uvec3 varying_position[];

uniform uvec2 min_max_depth_buffer;
uniform int current_level;

out uvec3 xfb_output;

void main() {

  if (current_level == (varying_position[0].z >> BIT_CURRENT_LEVEL)) {
    const uint old_layer = (varying_position[0].z >> BIT_CURRENT_LEVEL);
    const uint new_layer = old_layer - 1;
    const uint old_cellsize = 1 << old_layer;
    const uint new_cellsize = 1 << new_layer;

    uint continuity_flags = 0;

  #if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD

    float is_surface = texelFetch(sampler2D(min_max_depth_buffer), ivec2(varying_position[0].xy/old_cellsize), int(new_layer)).x;
    if (is_surface == 0) {

  #else

    uint bit_data = texelFetch(usampler2D(min_max_depth_buffer), ivec2(varying_position[0].xy/old_cellsize), int(new_layer)).x;
    continuity_flags = ALL_CONTINUITY_BITS & bit_data;

    if ((bit_data & ALL_MERGE_TYPE_BITS) == MERGE_NONE) {

  #endif

      // the current cell covers more than one surface --- we need to split it!

      const uvec2 offsets[4] = {uvec2(0),            uvec2(new_cellsize, 0),
                                uvec2(new_cellsize), uvec2(0, new_cellsize)};

      if (new_layer == 0) {
        // we are at the last level -- dont write one pixel sized quads; just output one quad (2x2 pixels) and set the is_surface flag to 0
        xfb_output = uvec3(varying_position[0].xy, ((new_layer << BIT_CURRENT_LEVEL) | continuity_flags | 0));
        EmitVertex(); EndPrimitive();
      } else {

        for (int v=0; v<4; ++v) {
          xfb_output = uvec3(varying_position[0].xy + offsets[v], (new_layer << BIT_CURRENT_LEVEL) | continuity_flags | int(new_layer==0));

          if (xfb_output.x < gua_resolution.x && xfb_output.y < gua_resolution.y) {
            EmitVertex(); EndPrimitive();
          }
        }
      }

    } else {


      #if WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION

        // if ((bit_data & ALL_MERGE_TYPE_BITS) == MERGE_LR) {

        //   xfb_output = uvec3(varying_position[0].xy, (old_layer << BIT_CURRENT_LEVEL) | (1<<BIT_EXPAND_Y) | continuity_flags | 1);
        //   EmitVertex(); EndPrimitive();
        //   xfb_output = uvec3(varying_position[0].xy+uvec2(new_cellsize, 0), (old_layer << BIT_CURRENT_LEVEL) | (1<<BIT_EXPAND_Y) | continuity_flags | 1);
        //   EmitVertex(); EndPrimitive();

        // } else if ((bit_data & ALL_MERGE_TYPE_BITS) == MERGE_L) {
        if ((bit_data & ALL_MERGE_TYPE_BITS) == MERGE_L) {

          xfb_output = uvec3(varying_position[0].xy, (old_layer << BIT_CURRENT_LEVEL) | (1<<BIT_EXPAND_Y) | continuity_flags | 1);
          EmitVertex(); EndPrimitive();
          xfb_output = uvec3(varying_position[0].xy+uvec2(new_cellsize, 0), (new_layer << BIT_CURRENT_LEVEL) | continuity_flags | int(new_layer==0));
          EmitVertex(); EndPrimitive();
          xfb_output = uvec3(varying_position[0].xy+uvec2(new_cellsize), (new_layer << BIT_CURRENT_LEVEL) | continuity_flags | int(new_layer==0));
          EmitVertex(); EndPrimitive();

        // } else if ((bit_data & ALL_MERGE_TYPE_BITS) == MERGE_R) {

        //   xfb_output = uvec3(varying_position[0].xy+uvec2(new_cellsize, 0), (old_layer << BIT_CURRENT_LEVEL) | (1<<BIT_EXPAND_Y) | continuity_flags | 1);
        //   EmitVertex(); EndPrimitive();
        //   xfb_output = uvec3(varying_position[0].xy, (new_layer << BIT_CURRENT_LEVEL) | continuity_flags | int(new_layer==0));
        //   EmitVertex(); EndPrimitive();
        //   xfb_output = uvec3(varying_position[0].xy+uvec2(0, new_cellsize), (new_layer << BIT_CURRENT_LEVEL) | continuity_flags | int(new_layer==0));
        //   EmitVertex(); EndPrimitive();

        // } else if ((bit_data & ALL_MERGE_TYPE_BITS) == MERGE_TB) {

        //   xfb_output = uvec3(varying_position[0].xy, (old_layer << BIT_CURRENT_LEVEL) | (1<<BIT_EXPAND_X) | continuity_flags | 1);
        //   EmitVertex(); EndPrimitive();
        //   xfb_output = uvec3(varying_position[0].xy+uvec2(0, new_cellsize), (old_layer << BIT_CURRENT_LEVEL) | (1<<BIT_EXPAND_X) | continuity_flags | 1);
        //   EmitVertex(); EndPrimitive();

        // } else if ((bit_data & ALL_MERGE_TYPE_BITS) == MERGE_T) {

        //   xfb_output = uvec3(varying_position[0].xy+uvec2(0, new_cellsize), (old_layer << BIT_CURRENT_LEVEL) | (1<<BIT_EXPAND_X) | continuity_flags | 1);
        //   EmitVertex(); EndPrimitive();
        //   xfb_output = uvec3(varying_position[0].xy, (new_layer << BIT_CURRENT_LEVEL) | continuity_flags | int(new_layer==0));
        //   EmitVertex(); EndPrimitive();
        //   xfb_output = uvec3(varying_position[0].xy+uvec2(new_cellsize, 0), (new_layer << BIT_CURRENT_LEVEL) | continuity_flags | int(new_layer==0));
        //   EmitVertex(); EndPrimitive();

        // } else if ((bit_data & ALL_MERGE_TYPE_BITS) == MERGE_B) {

        //   xfb_output = uvec3(varying_position[0].xy, (old_layer << BIT_CURRENT_LEVEL) | (1<<BIT_EXPAND_X) | continuity_flags | 1);
        //   EmitVertex(); EndPrimitive();
        //   xfb_output = uvec3(varying_position[0].xy+uvec2(new_cellsize), (new_layer << BIT_CURRENT_LEVEL) | continuity_flags | int(new_layer==0));
        //   EmitVertex(); EndPrimitive();
        //   xfb_output = uvec3(varying_position[0].xy+uvec2(0, new_cellsize), (new_layer << BIT_CURRENT_LEVEL) | continuity_flags | int(new_layer==0));
        //   EmitVertex(); EndPrimitive();

        } else {
          // the current cell covers only one continuous surface --- write it's size, continuity and that's one surface
          xfb_output = uvec3(varying_position[0].xy, ((old_layer << BIT_CURRENT_LEVEL) | continuity_flags | 1));
          EmitVertex(); EndPrimitive();

        }

      #else

        // the current cell covers only one continuous surface --- write it's size, continuity and that's one surface
        xfb_output = uvec3(varying_position[0].xy, ((old_layer << BIT_CURRENT_LEVEL) | continuity_flags | 1));
        EmitVertex(); EndPrimitive();

      #endif
    }

  } else {
    // the current cell has not been split in the previous pass --- we dont need to touch it
    xfb_output = varying_position[0];
    EmitVertex(); EndPrimitive();
  }
}
