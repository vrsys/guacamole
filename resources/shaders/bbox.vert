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

// input
layout(location=0) in vec3 gua_in_min;
layout(location=1) in vec3 gua_in_max;

@include "shaders/common/gua_camera_uniforms.glsl"

// output
out vec3 gua_min;
out vec3 gua_max;

out int is_for_right_eye; 
// body
void main() { 
    gua_min = gua_in_min;
    gua_max = gua_in_max;

    int viewport_index = 0;
#if @get_enable_multi_view_rendering@

  if(1 == gua_camera_in_multi_view_rendering_mode) {
    viewport_index = gl_InstanceID;
    is_for_right_eye = viewport_index;
  } 
  if(1 == gua_hardware_multi_view_rendering_mode_enabled) {
    viewport_index = int(gl_ViewID_OVR);
    is_for_right_eye = viewport_index;
  }
#endif
    

}
