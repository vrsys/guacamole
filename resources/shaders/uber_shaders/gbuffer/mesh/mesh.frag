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

// input from vertex shader ----------------------------------------------------
in vec3 gua_position_varying;
@input_definition

// uniforms
uniform bool gua_enable_global_clipping_plane;
uniform vec4 gua_global_clipping_plane;
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

// material specific uniforms
@uniform_definition

// outputs ---------------------------------------------------------------------
@output_definition

// methods ---------------------------------------------------------------------

// global gua_* methods

vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

uint gua_get_material_id() {
  return gua_uint_gbuffer_varying_0.x;
}

vec3 gua_get_position() {
  return gua_position_varying;
}

void gua_set_position(vec3 world_position) {
    vec4 pos = gua_projection_matrix * gua_view_matrix * vec4(world_position, 1.0);
    float ndc = pos.z/pos.w;
    gl_FragDepth = (((gl_DepthRange.diff) * ndc) + gl_DepthRange.near + gl_DepthRange.far) / 2.0;
}

void gua_clip_against_global_clipping_plane() {
  if (gua_enable_global_clipping_plane) {
    if (dot(gua_get_position(), gua_global_clipping_plane.xyz) + gua_global_clipping_plane.w < 0) {
      discard;
    }
  }
}

// material specific methods
@material_methods

// main ------------------------------------------------------------------------
void main() {
  gua_clip_against_global_clipping_plane();

  gl_FragDepth = gl_FragCoord.z;

  // big switch, one case for each material
  @material_switch

  gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;
}

