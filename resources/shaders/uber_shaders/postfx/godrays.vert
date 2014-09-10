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

@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"
//uniform mat4 gua_projection_matrix;
//uniform mat4 gua_view_matrix;
uniform vec3 gua_light_position_direction;

// output
out vec3 gua_light_position_screen_space;
out vec2 gua_quad_coords;

subroutine void CalculatePositionType();
subroutine uniform CalculatePositionType compute_position;

subroutine(CalculatePositionType)
void gua_calculate_by_direction() {
  vec4 tmp = gua_view_matrix * vec4(gua_light_position_direction, 0.0);

  if (tmp.z > 0) {
    // hide sun on wrong side
    gua_light_position_screen_space = vec3(-10);
  } else {

    tmp = gua_projection_matrix * tmp;
    gua_light_position_screen_space = (tmp/tmp.w).xyz;
    gua_light_position_screen_space = gua_light_position_screen_space/gua_light_position_screen_space.z;
  }
}

subroutine(CalculatePositionType)
void gua_calculate_by_position() {
  vec4 tmp = gua_projection_matrix * gua_view_matrix * vec4(gua_light_position_direction, 1.0);
  gua_light_position_screen_space = (tmp/tmp.w).xyz;
}

// body
void main() {
  gua_quad_coords = gua_in_texcoord;
  compute_position();
  gl_Position = vec4(gua_in_position, 1.0);
}
