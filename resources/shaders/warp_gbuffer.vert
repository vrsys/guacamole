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
@include "gbuffer_warp_modes.glsl"
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"

// -----------------------------------------------------------------------------
#if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD || WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_ADVANCED_SURFACE_ESTIMATION
// -----------------------------------------------------------------------------

layout(location=0) in uvec3 position;

flat out uvec3 varying_position;

void main() {
  varying_position = position;
}

// -----------------------------------------------------------------------------
#elif WARP_MODE == WARP_MODE_POINTS || WARP_MODE == WARP_MODE_SCALED_POINTS // -
// -----------------------------------------------------------------------------

layout(location=0) in float foo;

out vec3 color;
out vec3 normal;

uniform mat4 warp_matrix;

void main() {
  vec2 pos = vec2(gl_VertexID % gua_resolution.x, gl_VertexID / gua_resolution.x) + 0.5;
  vec2 tex_coords = pos/vec2(gua_resolution.x, gua_resolution.y);

  float depth = gua_get_depth(tex_coords);

  if (depth < 1) {
    color = gua_get_color(tex_coords);
    normal = gua_get_normal(tex_coords);
    vec3 screen_space_pos = vec3(tex_coords*2-1, depth);
    gl_Position = warp_matrix * vec4(screen_space_pos, 1);

    #if WARP_MODE == WARP_MODE_SCALED_POINTS
      gl_PointSize = 15*(1-gl_Position.z/gl_Position.w);
    #else
      gl_PointSize = 1;
    #endif
  } else {
    gl_Position = vec4(10, 10, 10, 1 + 0.000000000000001*foo);
    gl_PointSize = 0;
  }
}


// -----------------------------------------------------------------------------
#else // all other modes -------------------------------------------------------
// -----------------------------------------------------------------------------

layout(location=0) in float foo;

flat out uint vertex_id;
out float bar;

void main() {
  vertex_id = gl_VertexID;
  bar = foo;
}

#endif
