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

// input from gbuffer ----------------------------------------------------
uniform uvec2 volume_texture;
uniform uvec2 transfer_texture;
uniform float sampling_distance;
uniform vec3  volume_bounds;

uniform uvec2 gua_ray_entry_in;

in vec2 gua_quad_coords;

// uniforms
@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/gua_gbuffer_input.glsl"

// write outputs ---------------------------------------------------------------
layout(location=0) out vec4 gua_out_color;

// methods ---------------------------------------------------------------------
float get_depth_z(vec3 world_position) {
    vec4 pos = gua_projection_matrix * gua_view_matrix * vec4(world_position, 1.0);
    float ndc = pos.z/pos.w;
    return ((gl_DepthRange.diff * ndc) + gl_DepthRange.near + gl_DepthRange.far) / 2.0;

}

float get_depth_linear(float depth_buffer_d) {
  float ndc = (depth_buffer_d * 2.0 - gl_DepthRange.near - gl_DepthRange.far)/gl_DepthRange.diff;
  vec4 enit = vec4(gl_FragCoord.xy * 2.0 - vec2(1.0), ndc, 1.0);
  vec4 enit_inv = (gua_inverse_projection_view_matrix * enit);
  enit_inv /= enit_inv.w;
  return enit_inv.z;
}

bool inside_volume_bounds(const in vec3 sampling_position) {
    return (   all(greaterThanEqual(sampling_position, vec3(0.0)))
            && all(lessThanEqual(sampling_position, volume_bounds)));
}

vec4 get_raycast_color(vec3 gua_object_volume_position,
          float d_gbuffer, float d_volume) {

  mat4 gua_invers_model_matrix = inverse(gua_model_matrix);
  vec3 object_ray = normalize(gua_object_volume_position - (gua_invers_model_matrix * vec4(gua_camera_position, 1.0)).xyz);
  float d_step = abs(-1.0 * get_depth_linear(get_depth_z((gua_model_matrix * vec4(gua_object_volume_position + object_ray * sampling_distance, 1.0)).xyz)) - d_volume);
  
  vec3 obj_to_tex         = vec3(1.0) / volume_bounds;
  vec3 ray_increment      = object_ray * sampling_distance;
  vec3 sampling_pos       = gua_object_volume_position + ray_increment; // test, increment just to be sure we are in the volume
    
  bool inside_volume = inside_volume_bounds(sampling_pos);
      
  vec4 color = vec4(0.0);
    
  int d_steps = int(abs(d_volume - d_gbuffer) / d_step);
  int d_step_cur = 0;
  
  while (inside_volume && (d_step_cur < d_steps)) {
    ++d_step_cur;

    // get sample
    float s   = texture(sampler3D(volume_texture), sampling_pos * obj_to_tex).x;
    vec4  src = texture(sampler2D(transfer_texture), vec2(s, 0.5));

    // increment ray
    sampling_pos  += ray_increment;
    inside_volume  = inside_volume_bounds(sampling_pos) && (color.a < 0.99f);
    
    // compositing
    float omda_sa = (1.0f - color.a) * src.a;
    color.rgb += omda_sa * src.rgb;
    color.a   += omda_sa;
  }

  return color;
}


// main ------------------------------------------------------------------------
void main() {

  vec3 gua_object_volume_position = texture(sampler2D(gua_ray_entry_in), gua_quad_coords).xyz;
  vec3 gua_world_volume_position = (gua_model_matrix * vec4(gua_object_volume_position, 1.0)).xyz;

  float d_gbuffer = texture(sampler2D(gua_gbuffer_depth), gua_quad_coords).x;
  float d_volume = get_depth_z(gua_world_volume_position);

  d_gbuffer = -1.0 * get_depth_linear(d_gbuffer);
  d_volume = -1.0 * get_depth_linear(d_volume);
  
  // compose  
  if(d_gbuffer > d_volume &&
    (gua_object_volume_position.x != 0.00 || 
     gua_object_volume_position.y != 0.00 || 
     gua_object_volume_position.z != 0.00)) {

    gua_out_color = get_raycast_color(gua_object_volume_position, d_gbuffer, d_volume);

  } else {
    gua_out_color = vec4(0);
  }
}

