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

@include "shaders/common/mvr_less_header.glsl"

// varyings
in vec2 gua_quad_coords;

@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/gua_gbuffer_input.glsl"

// output
@include "shaders/common/gua_fragment_shader_output.glsl"

void main() {
  vec3 gua_color = vec3(1, 1, 1);
  float gua_emissivity = 1.0;
  float gua_roughness = 0.0;
  float gua_metalness = 0.0;
  vec3 gua_normal = vec3(0.0);
  bool gua_flags_passthrough = true;
  vec4 gua_uvs = vec4(0.0, 0.0, 0.0, 0.0);
  
  @include "shaders/common/gua_write_gbuffer.glsl"
}

