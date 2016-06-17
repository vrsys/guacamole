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

uniform sampler2DArray depth_texture;
uniform vec2 tex_size_inv;
uniform int layer;
uniform float cv_min_d;
uniform float cv_max_d;

in vec2 gua_quad_coords;

// write outputs
layout(location=0) out vec2 gua_out_color;

// bilateral filter
////////////////////////////////////////////////////////////////////
const int kernel_size = 6; // in pixel
const int kernel_end = kernel_size + 1;


const float dist_space_max_inv = 1.0/float(kernel_size);
float computeGaussSpace(float dist_space){
  float gauss_coord = dist_space * dist_space_max_inv;
  return 1.0 - gauss_coord;//texture2D(gauss,vec2(gauss_coord,0.5)).r;
}

float dist_range_max = 0.05; // in meter
float dist_range_max_inv = 1.0/dist_range_max;
float computeGaussRange(float dist_range){
  float gauss_coord = min(dist_range, dist_range_max) * dist_range_max_inv;
  return 1.0 - gauss_coord;//texture2D(gauss,vec2(gauss_coord,0.5)).r;
}

bool is_outside(float d){
  return (d < cv_min_d) || (d > cv_max_d);
}

vec2 bilateral_filter(vec3 coords){

  float depth = texture2DArray(depth_texture, coords).r;
  if(is_outside(depth)){
    return vec2(0.0,0.0);
  }

  // the valid range scales with depth
  float max_depth = 4.5; // Kinect V2
  float d_dmax = depth/max_depth;
  dist_range_max = 0.35 * d_dmax; // threshold around 
  dist_range_max_inv = 1.0/dist_range_max;

  float depth_bf = 0.0;

  float w = 0.0;
  float w_range = 0.0;
  float border_samples = 0.0;
  float num_samples = 0.0;
  for(int y = -kernel_size; y < kernel_end; ++y){
    for(int x = -kernel_size; x < kernel_end; ++x){
      num_samples += 1.0;
      vec3 coords_s = vec3(coords.s + float(x) * tex_size_inv.x, coords.t + float(y) * tex_size_inv.y, float(layer));
      
      const float depth_s = texture2DArray(depth_texture, coords_s).r;
      const float depth_range = abs(depth_s - depth);
      if(is_outside(depth_s) || (depth_range > dist_range_max)){
				border_samples += 1.0;
				continue;
      }

      float gauss_space = computeGaussSpace(length(vec2(x,y)));
      float gauss_range = computeGaussRange(depth_range);
      float w_s = gauss_space * gauss_range;
      depth_bf += w_s * depth_s;
      w += w_s;
      w_range += gauss_range;
    }
  }

  const float lateral_quality  = 1.0 - border_samples/num_samples;
  float filtered_depth = 0.0;
  if(w > 0.0)
    filtered_depth = depth_bf/w;
  else
    filtered_depth = 0.0;

#if 1
  if(w_range < (num_samples * 0.65)){
    filtered_depth = 0.0;
  }
#endif

  return vec2(filtered_depth, pow(lateral_quality,30));
}

void main() {
  gua_out_color = bilateral_filter(vec3(gua_quad_coords, layer));
}
