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

#if @get_enable_multi_view_rendering@
#extension GL_OVR_multiview2: require
layout(num_views = 2) in;
#endif

@include "common/gua_camera_uniforms.glsl"



// input
layout(location=0) in vec3 gua_in_position;
layout(location=2) in vec2 gua_in_texcoord;

// output
out vec2 gua_quad_coords;

// body
void main() {
#if @get_enable_multi_view_rendering@
  int viewport_index = 0;
  if(1 == gua_camera_in_multi_view_rendering_mode) {
    viewport_index = gl_InstanceID;
  //test_color = vec3(0.0, 1.0, 0.0);
  } 
  if(1 == gua_hardware_multi_view_rendering_mode_enabled) {
    viewport_index = int(gl_ViewID_OVR);
  //test_color = vec3(0.0, 0.0, 1.0);
  }

  gl_ViewportIndex = viewport_index;
#endif

  gua_quad_coords = gua_in_texcoord;
  gl_Position = vec4(gua_in_position, 1.0);
}
