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
uniform uvec2 warpR;
uniform uvec2 warpG;
uniform uvec2 warpB;

subroutine vec3 GetColorType();
subroutine uniform GetColorType get_color;

layout (location = 0) out vec3 out_color;

sampler2D get_tex(uvec2 handle) {
  return sampler2D(uint64_t(handle.x) + uint64_t(handle.y) * 4294967295);
}

vec3 warp_color(uvec2 image, vec2 tex_crd) {
  vec2 locR = texture2D( get_tex(warpR), tex_crd).rg;
  vec2 locG = texture2D( get_tex(warpG), tex_crd).rg;
  vec2 locB = texture2D( get_tex(warpB), tex_crd).rg;

  vec3 result = vec3(0.0, 0.0, 0.0);

  if(locR.r > 0.0) {
    result.r = texture2D( get_tex(image), locR).r;
  }

  if(locG.r > 0.0) {
    result.g = texture2D( get_tex(image), locG).g;
  }

  if(locB.r > 0.0) {
    result.b = texture2D( get_tex(image), locB).b;
  }

  return result;
}

subroutine( GetColorType )
vec3 get_red() {
  return vec3(warp_color(sampler, tex_coord).r, 0.0, 0.0);
}

subroutine( GetColorType )
vec3 get_green() {
  return vec3(0.0, warp_color(sampler, tex_coord).g, 0.0);
}

subroutine( GetColorType )
vec3 get_cyan() {
  return vec3(0.0, warp_color(sampler, tex_coord).gb);
}

subroutine( GetColorType )
vec3 get_full() {
  return warp_color(sampler, tex_coord);
}

void main() {
  out_color = get_color();
}

