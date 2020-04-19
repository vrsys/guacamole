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


#if 0 & @is_hardware_multi_view_rendering_enabled@
#extension GL_EXT_multiview_tessellation_geometry_shader: require
layout(num_views = 2) in;
#endif //is_hardware_multi_view_rendering_enabled

@include "shaders/common/gua_camera_uniforms.glsl"



layout(points) in;
layout(line_strip, max_vertices = 16) out;

in vec3 gua_min[];
in vec3 gua_max[];
in int is_for_right_eye[];
// body 
void main() {
mat4 mat;

#if @get_enable_multi_view_rendering@
    if (0 == is_for_right_eye[0]) {
      mat = gua_view_projection_matrix;
    } else {
      mat = gua_secondary_view_projection_matrix;
    }
#else 
    mat = gua_view_projection_matrix;
#endif

    gl_ViewportIndex  = is_for_right_eye[0];
    gl_Position = mat * vec4(gua_min[0].x, gua_min[0].y, gua_min[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_min[0].x, gua_min[0].y, gua_max[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_max[0].x, gua_min[0].y, gua_max[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_max[0].x, gua_min[0].y, gua_min[0].z, 1.0);
    EmitVertex(); 

    EndPrimitive();

    gl_ViewportIndex  = is_for_right_eye[0];
    gl_Position = mat * vec4(gua_min[0].x, gua_min[0].y, gua_max[0].z, 1.0);
    EmitVertex(); 
    
    gl_Position = mat * vec4(gua_min[0].x, gua_max[0].y, gua_max[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_max[0].x, gua_max[0].y, gua_max[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_max[0].x, gua_min[0].y, gua_max[0].z, 1.0);
    EmitVertex(); 
    EndPrimitive();

    gl_ViewportIndex  = is_for_right_eye[0];
    gl_Position = mat * vec4(gua_min[0].x, gua_max[0].y, gua_max[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_min[0].x, gua_max[0].y, gua_min[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_max[0].x, gua_max[0].y, gua_min[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_max[0].x, gua_max[0].y, gua_max[0].z, 1.0);
    EmitVertex(); 
    EndPrimitive();

    gl_ViewportIndex  = is_for_right_eye[0];
    gl_Position = mat * vec4(gua_min[0].x, gua_max[0].y, gua_min[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_min[0].x, gua_min[0].y, gua_min[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_max[0].x, gua_min[0].y, gua_min[0].z, 1.0);
    EmitVertex(); 

    gl_Position = mat * vec4(gua_max[0].x, gua_max[0].y, gua_min[0].z, 1.0);
    EmitVertex(); 
    EndPrimitive();
}
