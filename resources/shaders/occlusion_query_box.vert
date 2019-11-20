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

// uniforms
@include "shaders/common/gua_camera_uniforms.glsl"


layout(location=0) in vec3 in_position;
//ignore remaing layout parameters

uniform mat4 view_projection_matrix;
uniform vec3 world_space_bb_min;
uniform vec3 world_space_bb_max;

// simplest possible vertex shader that does not use constant values
void main() {

  vec3 bounding_box_dims = world_space_bb_max - world_space_bb_min;


  gl_Position = view_projection_matrix * vec4(bounding_box_dims * in_position + world_space_bb_min, 1.0);
}
