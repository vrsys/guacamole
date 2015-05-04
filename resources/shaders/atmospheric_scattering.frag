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

in flat int texlayer;
in vec2 texcoods;

uniform float texel_size;
uniform vec3 ground_color;
uniform vec3 light_direction;
uniform vec3 light_color;

// write outputs
layout(location=0) out vec3 gua_out_color;

vec3 get_view_direction() {
  vec2 frag_coord = gl_FragCoord.xy * texel_size -0.5;

  switch(texlayer) {
    case 0: return normalize(vec3(0.5,-frag_coord.y, -frag_coord.x));
    case 1: return normalize(vec3(-0.5,-frag_coord.y, frag_coord.x));
    case 2: return normalize(vec3(frag_coord.x, 0.5, frag_coord.y));
    case 3: return normalize(vec3(frag_coord.x, -0.5, -frag_coord.y));
    case 4: return normalize(vec3(frag_coord.x, -frag_coord.y, 0.5));
    case 5: return normalize(vec3(-frag_coord.x, -frag_coord.y, -0.5));
  }

  return vec3(0,0,0);
}


// Code for Rayleigh/Mie-Scattering adopted from Nathaniel Meyer's
// implementation of  Sean O'Neill's GPU Gems 2 article.

#ifdef GL_ES
  precision highp float;
#endif

// atmosphere properties
const vec3 center = vec3(0);      // center position of planet
const float inner_radius = 1.0;    // Radius of planet
const float outer_radius = inner_radius * 1.025;    // Outter radius of atmosphere (= inner_radius * 1.025)
const float num_samples = 5.0;   // Number of scatter samples to perform

const vec3 wave_length = 1.0 / pow(light_color, vec3(4.0));

// constants
const float PI = 3.14159265358979323846;
const float Kr4PI = 0.0025 * 4 * PI;  // Kr * 4 * PI
const float KrESun = 0.0025 * 15; // Kr * ESun
const float Km4PI = 0.0005 * 4 * PI;  // Km * 4 * PI
const float KmESun = 0.0005 * 15; // Km * ESun
const float MiePhase = -0.95; // Used in phase function, not applicable for Rayleigh

// gamma
const float InvGamma = 1/2.2;

bool intersect_ray_sphere(in vec3 p, in vec3 v, in vec3 centre, in float radius,
                          out vec3 intersection_near, out vec3 intersection_far) {
  vec3 vc = p - centre;
  float B = dot(-vc, v);
  float C = dot(vc, vc) - (radius * radius);
  float D = (B * B) - C;

  // Calculate intersection point
  if ( D >= 0.0 )
  {
    D = sqrt(D);

    float t0 = B - D;
    float t1 = B + D;

    // Smallest positive root is the closest intersection point.
    if ( (t1 >= 0.0) && ((t1 < t0) || (t0 < 0.0)) )
    {
      intersection_near = p + v * t1;
      intersection_far = p + v * t0;
    }
    else if ( t0 >= 0.0 )
    {
      intersection_near = p + v * t0;
      intersection_far = p + v * t1;
    }
    else
      return false;

    return true;
  }

  return false;
}


float scale_angle(float cose) {
  const float height_density = 0.25;

  float x = 1.0 - cose;
  return height_density * exp(-0.00287 + x * (0.459 + x * (3.83 + x * (-6.80 + x * 5.25))));
}

float get_phase(float cose, float cos2, float g, float g2) {
  return (1.5 * ((1.0 - g2) / (2.0 + g2))) * ((1.0 + cos2) / pow(1.0 + g2 - 2.0 * g * cose, 1.5));
}

float get_rayleigh_phase(float cos2) {
   return 0.75 * (1.0 + cos2);
}


void main() {
  const float height_density = 0.25;

  vec3 color = vec3(0);
  vec3 direction = get_view_direction();
  vec3 position = vec3(0,1,0);

  // Find intersection points in the atmosphere
  vec3 intersection_near;
  vec3 intersection_far;
  if (intersect_ray_sphere(position, direction, center, outer_radius, intersection_near, intersection_far) ) {
    // An intersection was found. Compute scattering effect.

    // Are we inside or outside the atmosphere?
    vec3 start_point;
    vec3 end_point;
    float height = distance(position, center);
    bool inside = height < outer_radius;

    if (inside) {
      // Inside scattering
      start_point = position;
      end_point = intersection_near;
    } else {
      // Outside scattering
      start_point = intersection_near;
      end_point = intersection_far;
    }

    // Setup variables
    float dist = distance(start_point, end_point);
    float scale = 1.0 / (outer_radius - inner_radius);
    float scale_over_scale_depth = scale / height_density;


    // Starting scatter
    height = length(start_point);
    float start_angle = dot(direction, start_point) / (inside ? height : outer_radius);
    float start_depth = inside ? exp(scale_over_scale_depth * (inner_radius - height)) : exp(-1.0 / height_density);
    float start_offset = start_depth * scale_angle(start_angle);

    // Initialize the scattering loop variables
    float sample_length = dist / num_samples;
    float scaled_length = sample_length * scale;
    vec3 sample_ray = direction * sample_length;
    vec3 sample_point = start_point + sample_ray * 0.5;


    // Iterate through sample points
    for (float i = 0.0; i < 10.0; ++i)
    {
      if ( i >= num_samples )
        break;

      // Find height, depth, and angles at sample point
      height = length(sample_point);
      float depth = exp(scale_over_scale_depth * (inner_radius - height));
      float light_angle = dot(-light_direction, sample_point) / height;
      float camera_angle = dot(direction, sample_point) / height;

      // Scatter and attenuate light
      float scatter = (start_offset + depth * (scale_angle(light_angle) - scale_angle(camera_angle)));
      vec3 attenuate = exp(-scatter * (wave_length * Kr4PI + Km4PI));
      color += attenuate * (depth * scaled_length);

      // Move to the next sample point
      sample_point += sample_ray;
    }


    // Scale the Mie and Rayleigh colors
    vec3 mie_color = color * KmESun;
    vec3 rayleigh_color = color * (wave_length * KrESun);

    // Add rayleigh and mie scatter phase
    float cose = dot(light_direction, direction) / length(direction);
    float cos2 = cose * cose;
    color = (rayleigh_color * get_rayleigh_phase(cos2)) + (mie_color * get_phase(cose, cos2, MiePhase, MiePhase * MiePhase));
    color = clamp(color, 0, 1);

    if (start_angle < 0 && start_angle > -0.1) {
      color = mix(color, ground_color, abs(start_angle) * 10.0);
    } else if (start_angle <= -0.1) {
      color = ground_color;
    }
  }

  // Gamma correction
  color = pow(color, vec3(InvGamma));


  gua_out_color = color;
}
