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

in vec2 tex_coord;

uniform uvec2 sampler;

subroutine vec3 GetColorType();
subroutine uniform GetColorType get_color;

layout (location = 0) out vec3 out_color;

sampler2D get_tex(uvec2 handle) {
  return sampler2D(uint64_t(handle.x) + uint64_t(handle.y) * 4294967295);
}

subroutine( GetColorType )
vec3 get_red() {
  return vec3(texture2D( get_tex(sampler), tex_coord).r, 0.0, 0.0);
}

subroutine( GetColorType )
vec3 get_green() {
  return vec3(0.0, texture2D( get_tex(sampler), tex_coord).g, 0.0);
}

subroutine( GetColorType )
vec3 get_cyan() {
  return vec3(0.0, texture2D( get_tex(sampler), tex_coord).gb);
}

subroutine( GetColorType )
vec3 get_checker_even() {
  if (mod(gl_FragCoord.x + gl_FragCoord.y, 2.0) == 0.0)
    return vec3(texture2D( get_tex(sampler), tex_coord).rgb);
  else discard;
}

subroutine( GetColorType )
vec3 get_checker_odd() {
  if (mod(gl_FragCoord.x + gl_FragCoord.y, 2.0) == 1.0)
    return vec3(texture2D( get_tex(sampler), tex_coord).rgb);
  else discard;
}

subroutine( GetColorType )
vec3 get_full() {
  return vec3(texture2D( get_tex(sampler), tex_coord).rgb);
}

void main() {
  out_color = get_color();
}
