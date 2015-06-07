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

// -----------------------------------------------------------------------------
#if WARP_MODE == WARP_MODE_GRID || WARP_MODE == WARP_MODE_ADAPTIVE_GRID // -----
// -----------------------------------------------------------------------------

layout(location=0) in uvec3 position;

flat out uvec3 varying_position;

void main() {
  varying_position = position;
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