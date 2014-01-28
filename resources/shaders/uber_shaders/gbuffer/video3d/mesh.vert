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
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_texture_array : enable

// input -----------------------------------------------------------------------
layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;

// uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

uniform uint gua_material_id;

//calibration matrices
uniform mat4  image_d_to_eye_d;
uniform mat4  eye_d_to_world;
uniform mat4  eye_d_to_eye_rgb;
uniform mat4  eye_rgb_to_image_rgb;

//kinect depths
uniform sampler2DArray depth_video3d_texture;
uniform sampler2DArray color_video3d_texture;

// material specific uniforms
@uniform_definition

// outputs ---------------------------------------------------------------------
out vec3 gua_position_varying;

@output_definition

// global variables
vec2 gua_texcoords;

vec3 gua_world_normal;
vec3 gua_world_position;
vec3 gua_world_tangent;
vec3 gua_world_bitangent;

vec3 gua_object_normal;
vec3 gua_object_position;
vec2 gua_object_texcoords;
vec3 gua_object_tangent;
vec3 gua_object_bitangent;

// methods ---------------------------------------------------------------------

// global gua_* methods
@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

uint gua_get_material_id() {
  return gua_material_id;
}

// material specific methods
@material_methods

// main ------------------------------------------------------------------------
void main() {
  float depth = texture2DArray(depth_video3d_texture, vec3(gua_in_texcoords.xy, 0)).r; 

  vec4 POS_d = depth * image_d_to_eye_d * vec4(gua_in_position.xy, depth, 1.0);
  POS_d.z = depth;

  POS_d.w = 1.0;
  vec3 pos_d   = POS_d.xyz;

  vec4 POS_rgb = eye_d_to_eye_rgb * POS_d;

  if(POS_rgb.z > 0.0)
      gua_texcoords = (eye_rgb_to_image_rgb * vec4( (POS_rgb.xy/POS_rgb.z) ,0.0,1.0)).xy;
  else
      gua_texcoords = vec2(0.0);

  vec4 POS_ws =  eye_d_to_world * POS_d;

  if(POS_ws.y < 0.0)
    POS_ws.y = 0.0;

  gua_world_position = (gua_model_matrix * POS_ws).xyz;
  
  ////////////////////////////////////////////////////////////////////////////////////
  gua_position_varying = vec3(0);

  gua_texcoords = gua_in_texcoords;

  gua_object_normal =     gua_in_normal;
  gua_object_tangent =    gua_in_tangent;
  gua_object_bitangent =  gua_in_bitangent;
  gua_object_position =   POS_ws.xyz;
  if(depth > 0)
   gua_object_position =   vec3(0.0);//vec3(gua_in_position.xy, sin(depth));
  else
   gua_object_position =   gua_in_position;


  gua_world_normal =      normalize((gua_normal_matrix * vec4(gua_in_normal, 0.0)).xyz);
  gua_world_tangent =     normalize((gua_normal_matrix * vec4(gua_in_tangent, 0.0)).xyz);
  gua_world_bitangent =   normalize((gua_normal_matrix * vec4(gua_in_bitangent, 0.0)).xyz);
  if(depth > 0)
   gua_world_position =    (gua_model_matrix * vec4(vec3(0.0), 1.0)).xyz;//(gua_model_matrix * vec4(gua_in_position.xy, sin(depth), 1.0)).xyz;
  else
    gua_world_position =    (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;

  // big switch, one case for each material
  @material_switch

  gua_uint_gbuffer_varying_0.x = gua_material_id;
  gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_world_position.xyz, 1.0);
}
