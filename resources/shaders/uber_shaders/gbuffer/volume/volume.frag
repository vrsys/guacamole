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
in vec3 object_position_varying;
in vec3 object_normal_varying;
in vec3 object_world_normal_varying;
in vec3 object_ray;
@input_definition

// uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"
uniform sampler2D transfer_texture;
uniform sampler3D volume_texture;
uniform float sampling_distance;

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

vec3 get_gradient(const in vec3 sampling_pos, 
                  const in float f_distance, 
                  const in vec3 obj_to_tex, 
                  const in sampler3D volume_texture)
{

    float grad_x = texture(volume_texture, (sampling_pos - vec3(f_distance, 0.0, 0.0 )) * obj_to_tex).r 
                 - texture(volume_texture, (sampling_pos + vec3(f_distance, 0.0, 0.0 )) * obj_to_tex).r;
    float grad_y = texture(volume_texture, (sampling_pos - vec3(0.0, f_distance, 0.0 )) * obj_to_tex).r 
                 - texture(volume_texture, (sampling_pos + vec3(0.0, f_distance, 0.0 )) * obj_to_tex).r;
    float grad_z = texture(volume_texture, (sampling_pos - vec3(0.0, 0.0, f_distance )) * obj_to_tex).r 
                 - texture(volume_texture, (sampling_pos + vec3(0.0, 0.0, f_distance )) * obj_to_tex).r;
    
    
    return(vec3(grad_x, grad_y, grad_z) * 0.5f);
}


vec4 
get_shading(in vec3 spos, in vec3 grad, in vec4 color)
{
    //vec3 col = vec3(0.0, 0.0, 0.0);
    
    //vec3 lt = (vec4(light_pos, 0.0) * volume_data.mv_matrix).rgb;
    vec3 lt = vec3(0.0,0.0,0.0);//light_pos;

    vec3 n = normalize(grad);
    vec3 l = normalize(lt - spos);

    //vec3 e = camera_location.xyz;
    //vec3 v = normalize(e - spos);
    //float ks = 0.8;

    //vec3 r = (2 * n * dot(n, l)) - l;
        
    //float color_shini = ks * pow(dot(r,v), 3);

    //color.rgb = color.rgb * 0.3 + color.a * color.rgb * light_color * (dot(n, l) * 0.5f) + color_shini;
    //color.rgb =  light_color * (dot(n, l) * 0.5f + 0.5f);
    
    //return (vec4(col, color.a));
    return color;
}

bool
inside_volume_bounds(const in vec3 sampling_position)
{
    return (   all(greaterThanEqual(sampling_position, vec3(0.0)))
            && all(lessThanEqual(sampling_position, vec3(1.0))));
}

// material specific methods
@material_methods

// main ------------------------------------------------------------------------
void main() {

  gl_FragDepth = gl_FragCoord.z;

  // big switch, one case for each material
  @material_switch

  vec3 ray_entry_position = object_position_varying;
  //vec3 ray_increment      = normalize(ray_entry_position - gua_camera_position) * sampling_distance;
  vec3 ray_increment      = object_ray * sampling_distance;
  vec3 sampling_pos       = ray_entry_position + ray_increment; // test, increment just to be sure we are in the volume
    
  bool inside_volume = inside_volume_bounds(sampling_pos);
    
  bool iso_hit = false;

  vec3 normal = vec3(0.0);
  vec4 color = vec4(0.0);

  int step_count = 0;

#if 0
while (inside_volume && !iso_hit) 
{
      // get sample
      //float s = texture(volume_texture, sampling_pos * obj_to_tex).r;
      float s = texture(volume_texture, sampling_pos).r;
      
      // increment ray
      sampling_pos  += ray_increment;
      inside_volume  = inside_volume_bounds(sampling_pos);

      step_count++;
              
      if(s > 0.6)
      {
          iso_hit = true;

          //vec3 gradient = get_gradient(sampling_pos, 4.0 * sampling_distance, obj_to_tex, volume_texture);
          //normal = normalize(gradient);
      }
  }

	if(!iso_hit)
		gua_float_gbuffer_out_1.xyz = vec3(step_count/255.f);
	else
		gua_float_gbuffer_out_1.xyz = vec3(0.0, 1.0, 0.0);
#endif

#if 1
      while (inside_volume) {
        // get sample
        float s = texture(volume_texture, sampling_pos).r;
        vec4 src = texture(transfer_texture, vec2(s, 0.5));

        // increment ray
        sampling_pos  += ray_increment;
        inside_volume  = inside_volume_bounds(sampling_pos) && (color.a < 0.99);
        // compositing
        float omda_sa = (1.0 - color.a) * src.a;
        color.rgb += omda_sa * src.rgb;
        color.a   += omda_sa;
    }

	gua_float_gbuffer_out_1.rgb = color.rgb;

#endif
  
  vec4 trans_c = texture(transfer_texture, object_position_varying.xy);
  
  //gua_float_gbuffer_out_1 = trans_c.rgb * trans_c.a + object_position_varying.xyz * 0.5;

  gua_float_gbuffer_out_0.xyz = normal;
    
  gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;
}

