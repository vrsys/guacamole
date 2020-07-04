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

// uniforms
@include "shaders/common/gua_camera_uniforms.glsl"

// output
out vec2 gua_varying_quad_coords;
out vec3 gua_varying_normal;

// body
void main() {
    gua_varying_quad_coords = gua_in_texcoord;


#if @get_enable_multi_view_rendering@
    gl_Layer = gl_InstanceID;
    if(0 == gl_InstanceID) {
#endif
    	gua_varying_normal = (gua_normal_matrix * vec4(0.0, 0.0, 1.0, 0.0)).xyz;
    	gl_Position = gua_view_projection_matrix * gua_model_matrix * vec4(gua_in_position*0.5, 1.0);

#if @get_enable_multi_view_rendering@
} else {
    	gua_varying_normal = (gua_normal_matrix * vec4(0.0, 0.0, 1.0, 0.0)).xyz;
    	gl_Position = gua_secondary_view_projection_matrix * gua_model_matrix * vec4(gua_in_position*0.5, 1.0);		
	}
#endif
}
