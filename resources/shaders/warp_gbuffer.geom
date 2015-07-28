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

@include "common/header.glsl"
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"
@include "gbuffer_warp_modes.glsl"

uniform mat4 warp_matrix;

float gua_get_depth_raw(vec2 frag_pos) {
  return texelFetch(sampler2D(gua_gbuffer_depth), ivec2(frag_pos), 0).x;
}

// -----------------------------------------------------------------------------
#if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD || WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_ADVANCED_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION
// -----------------------------------------------------------------------------

#define GAP @pixel_size@

layout(points) in;
layout(triangle_strip, max_vertices = 16) out;

@include "shaders/warp_grid_bits.glsl"

flat in  uvec3 varying_position[];
flat out uint cellsize;

out vec2 texcoords;

void emit_grid_vertex(vec2 position, float depth) {
  texcoords = position / gua_resolution;
  gl_Position = warp_matrix * vec4(2.0 * texcoords - 1.0, depth, 1.0);
  EmitVertex();
}

void emit_quad(uvec2 offset, uvec2 size) {


  if (size.x > 0 && size.y > 0) {

    cellsize = min(size.x, size.y);
    float depth;
    vec2 position;

    #if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD
      position = varying_position[0].xy+offset;
      depth = gua_get_depth_raw(position);
      emit_grid_vertex(position + vec2(0, 0) + vec2(-GAP, -GAP), depth);

      position = varying_position[0].xy+offset + vec2(size.x-1, 0);
      depth = gua_get_depth_raw(position);
      emit_grid_vertex(position + vec2(1, 0) + vec2(GAP, -GAP), depth);

      position = varying_position[0].xy+offset + vec2(0, size.y-1);
      depth = gua_get_depth_raw(position);
      emit_grid_vertex(position + vec2(0, 1) + vec2(-GAP, GAP), depth);

      position = varying_position[0].xy+offset + vec2(size.x-1, size.y-1);
      depth = gua_get_depth_raw(position);
      emit_grid_vertex(position + vec2(1, 1) + vec2(GAP, GAP), depth);

    #elif WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION

      position = varying_position[0].xy+offset + vec2(0, 0)  + vec2(-GAP, -GAP);
      depth = gua_get_unscaled_depth(position/gua_resolution);
      emit_grid_vertex(position, depth);

      position = varying_position[0].xy+offset + vec2(size.x, 0)  + vec2( GAP, -GAP);
      depth = gua_get_unscaled_depth(position/gua_resolution);
      emit_grid_vertex(position, depth);

      position = varying_position[0].xy+offset + vec2(0, size.y)  + vec2(-GAP,  GAP);
      depth = gua_get_unscaled_depth(position/gua_resolution);
      emit_grid_vertex(position, depth);

      position = varying_position[0].xy+offset + vec2(size.x, size.y)  + vec2( GAP,  GAP);
      depth = gua_get_unscaled_depth(position/gua_resolution);
      emit_grid_vertex(position, depth);

    #elif WARP_MODE == WARP_MODE_GRID_ADVANCED_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION

      const int cont_l = int(varying_position[0].z >> BIT_CONTINUOUS_L) & 1;
      const int cont_r = int(varying_position[0].z >> BIT_CONTINUOUS_R) & 1;
      const int cont_t = int(varying_position[0].z >> BIT_CONTINUOUS_T) & 1;
      const int cont_b = int(varying_position[0].z >> BIT_CONTINUOUS_B) & 1;

      const int cont_tl = int(varying_position[0].z >> BIT_CONTINUOUS_TL) & 1;
      const int cont_tr = int(varying_position[0].z >> BIT_CONTINUOUS_TR) & 1;
      const int cont_bl = int(varying_position[0].z >> BIT_CONTINUOUS_BL) & 1;
      const int cont_br = int(varying_position[0].z >> BIT_CONTINUOUS_BR) & 1;

      position = varying_position[0].xy+offset;
      vec2 lookup_offset = vec2(-cont_l, -cont_b) * cont_bl;
      depth = gua_get_unscaled_depth( (position + 0.5*(1+lookup_offset)) / gua_resolution);
      emit_grid_vertex(position + vec2(0, 0) + vec2(-GAP, -GAP), depth);

      position = varying_position[0].xy+offset + vec2(size.x-1, 0);
      lookup_offset = vec2(cont_r, -cont_b) * cont_br;
      depth = gua_get_unscaled_depth( (position + 0.5*(1+lookup_offset)) / gua_resolution);
      emit_grid_vertex(position + vec2(1, 0) + vec2(GAP, -GAP), depth);

      position = varying_position[0].xy+offset + vec2(0, size.y-1);
      lookup_offset = vec2(-cont_l, cont_t) * cont_tl;
      depth = gua_get_unscaled_depth( (position + 0.5*(1+lookup_offset)) / gua_resolution);
      emit_grid_vertex(position + vec2(0, 1) + vec2(-GAP, GAP), depth);

      position = varying_position[0].xy+offset + vec2(size.x-1, size.y-1);
      lookup_offset = vec2(cont_r, cont_t) * cont_tr;
      depth = gua_get_unscaled_depth( (position + 0.5*(1+lookup_offset)) / gua_resolution);
      emit_grid_vertex(position + vec2(1, 1) + vec2(GAP, GAP), depth);

    #endif // ------------------------------------------------------------------

    EndPrimitive();
  }
}

void emit_pixel(uvec2 offset) {

  const float depth = gua_get_depth_raw(varying_position[0].xy + offset);

  cellsize = 1;
  emit_grid_vertex(varying_position[0].xy + offset + vec2(0, 0) + vec2(-GAP, -GAP), depth);
  emit_grid_vertex(varying_position[0].xy + offset + vec2(1, 0) + vec2( GAP, -GAP), depth);
  emit_grid_vertex(varying_position[0].xy + offset + vec2(0, 1) + vec2(-GAP,  GAP), depth);
  emit_grid_vertex(varying_position[0].xy + offset + vec2(1, 1) + vec2( GAP,  GAP), depth);
  EndPrimitive();
}

void emit_pixel(uvec2 offset, uint do_emit) {

  if (do_emit > 0) {
    cellsize = 1;
    const float depth = gua_get_depth_raw(varying_position[0].xy + offset);
    emit_grid_vertex(varying_position[0].xy + offset + vec2(0, 0) + vec2(-GAP, -GAP), depth);
    emit_grid_vertex(varying_position[0].xy + offset + vec2(1, 0) + vec2( GAP, -GAP), depth);
    emit_grid_vertex(varying_position[0].xy + offset + vec2(0, 1) + vec2(-GAP,  GAP), depth);
    emit_grid_vertex(varying_position[0].xy + offset + vec2(1, 1) + vec2( GAP,  GAP), depth);
    EndPrimitive();
  }
}

void main() {

#if WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION

  const uint merge_type = varying_position[0].z & ALL_MERGE_TYPE_BITS;

  uvec4 quad0 = uvec4(0);
  uvec4 quad1 = uvec4(0);
  uvec4 quad2 = uvec4(0);
  uvec4 quad3 = uvec4(0);

  if ((varying_position[0].z & 1) > 0) {
    uvec2 scale = 1 + uvec2((varying_position[0].z >> BIT_EXPAND_X) & 1, (varying_position[0].z >> BIT_EXPAND_Y) & 1);
    quad0 = uvec4(uvec2(0), (1 << (varying_position[0].z >> BIT_CURRENT_LEVEL)) * scale);

  } else if (merge_type == MERGE_NONE) {
    quad0 = uvec4(0, 0, 1, 1);
    quad1 = uvec4(1, 0, 1, 1);
    quad2 = uvec4(1, 1, 1, 1);
    quad3 = uvec4(0, 1, 1, 1);

  } else if (merge_type == MERGE_LR) {
    quad0 = uvec4(uvec2(0, 0), uvec2(1, 2));
    quad1 = uvec4(uvec2(1, 0), uvec2(1, 2));

  } else if (merge_type == MERGE_L) {
    quad0 = uvec4(uvec2(0, 0), uvec2(1, 2));
    quad1 = uvec4(1, 0, 1, 1);
    quad2 = uvec4(1, 1, 1, 1);

  } else if (merge_type == MERGE_R) {
    quad0 = uvec4(uvec2(1, 0), uvec2(1, 2));
    quad1 = uvec4(0, 0, 1, 1);
    quad2 = uvec4(0, 1, 1, 1);

  } else if (merge_type == MERGE_TB) {
    quad0 = uvec4(uvec2(0, 0), uvec2(2, 1));
    quad1 = uvec4(uvec2(0, 1), uvec2(2, 1));

  } else if (merge_type == MERGE_T) {
    quad0 = uvec4(uvec2(0, 1), uvec2(2, 1));
    quad1 = uvec4(0, 0, 1, 1);
    quad2 = uvec4(1, 0, 1, 1);

  } else if (merge_type == MERGE_B) {
    quad0 = uvec4(uvec2(0, 0), uvec2(2, 1));
    quad1 = uvec4(1, 1, 1, 1);
    quad2 = uvec4(0, 1, 1, 1);

  }

  emit_quad(quad0.xy, quad0.zw);
  emit_quad(quad1.xy, quad1.zw);
  emit_pixel(quad2.xy, quad2.z);
  emit_pixel(quad3.xy, quad3.z);

#elif WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION

  if ((varying_position[0].z & 1) > 0) {
    emit_quad(uvec2(0), uvec2(1 << (varying_position[0].z >> BIT_CURRENT_LEVEL)));
  } else {
    emit_quad(uvec2(0, 0), uvec2(1 << (varying_position[0].z >> BIT_CURRENT_LEVEL)));
    emit_quad(uvec2(1, 0), uvec2(1 << (varying_position[0].z >> BIT_CURRENT_LEVEL)));
    emit_quad(uvec2(1, 1), uvec2(1 << (varying_position[0].z >> BIT_CURRENT_LEVEL)));
    emit_quad(uvec2(0, 1), uvec2(1 << (varying_position[0].z >> BIT_CURRENT_LEVEL)));
  }
  

#else
  
  if ((varying_position[0].z & 1) > 0) {
    emit_quad(uvec2(0), uvec2(1 << (varying_position[0].z >> BIT_CURRENT_LEVEL)));
  } else {
    emit_pixel(uvec2(0, 0));
    emit_pixel(uvec2(1, 0));
    emit_pixel(uvec2(1, 1));
    emit_pixel(uvec2(0, 1));
  }

#endif
}


// -----------------------------------------------------------------------------
#else // all other modes -------------------------------------------------------
// -----------------------------------------------------------------------------

layout(points) in;

layout(triangle_strip) out;
layout(max_vertices = 4) out;

flat in uint vertex_id[];
in float bar[];

out vec3 color;
out vec3 normal;


void emit_primitive(vec2 tex_coords) {
  float depth = gua_get_unscaled_depth(tex_coords);
  vec2 frag_pos = tex_coords*2-1;

  #if WARP_MODE == WARP_MODE_QUADS_SCREEN_ALIGNED
    const vec2 half_pixel = vec2(1.0) / vec2(gua_resolution);
    const vec2 offsets[4] = {vec2(half_pixel), vec2(-half_pixel.x, half_pixel.y),
                              vec2(half_pixel.x, -half_pixel.y), vec2(-half_pixel)};

    for (int v=0; v<4; ++v) {
      vec3 screen_space_pos = vec3(frag_pos + offsets[v], depth);
      gl_Position = warp_matrix * vec4(screen_space_pos, 1 + 0.000000000000001*bar[0]);

      EmitVertex();
    }

    EndPrimitive();

  #elif WARP_MODE == WARP_MODE_QUADS_NORMAL_ALIGNED

    const vec2 half_pixel = vec2(1.0) / vec2(gua_resolution);
    const vec2 offsets[4] = {vec2(half_pixel), vec2(-half_pixel.x, half_pixel.y),
                              vec2(half_pixel.x, -half_pixel.y), vec2(-half_pixel)};

    // normal = (transpose(gua_inverse_projection_matrix) * vec4(gua_get_normal(tex_coords), 0)).xyz;
    // vec3 pos = vec3(frag_pos, depth);
    // vec3 direction = pos;

    color = gua_get_normal(tex_coords);


    for (int v=0; v<4; ++v) {
      vec3 screen_space_pos = vec3(frag_pos + offsets[v], depth);
      gl_Position = warp_matrix * vec4(screen_space_pos, 1 + 0.000000000000001*bar[0]);

      EmitVertex();
    }

    EndPrimitive();

  #elif WARP_MODE == WARP_MODE_QUADS_DEPTH_ALIGNED

    float d_max = -1;
    vec2 t = tex_coords;

    vec2  p1 = vec2(tex_coords*gua_resolution) + vec2(0, 0);
    float d1 = gua_get_depth_raw(p1);

    if (d1 < 1 && d1 > d_max) {
      d_max = d1;
    }

    vec2  p2 = vec2(tex_coords*gua_resolution) + vec2(1, 0);
    float d2 = gua_get_depth_raw(p2);

    if (d2 < 1 && d2 > d_max) {
      d_max = d2;
      t = tex_coords + vec2(1.0/float(gua_resolution.x), 0);
    }

    vec2  p3 = vec2(tex_coords*gua_resolution) + vec2(0, 1);
    float d3 = gua_get_depth_raw(p3);

    if (d3 < 1 && d3 > d_max) {
      d_max = d3;
      t = tex_coords + vec2(0, 1.0/float(gua_resolution.y));
    }

    vec2  p4 = vec2(tex_coords*gua_resolution) + vec2(1, 1);
    float d4 = gua_get_depth_raw(p4);

    if (d4 < 1 && d4 > d_max) {
      d_max = d4;
      t = tex_coords + vec2(1.0)/vec2(gua_resolution);
    }

    color = gua_get_color(t);
    normal = gua_get_normal(t);

    gl_Position = warp_matrix * vec4(p1/gua_resolution*2-1, min(d_max, d1), 1 + 0.000000000000001*bar[0]);
    EmitVertex();

    gl_Position = warp_matrix * vec4(p2/gua_resolution*2-1, min(d_max, d2), 1 + 0.000000000000001*bar[0]);
    EmitVertex();

    gl_Position = warp_matrix * vec4(p3/gua_resolution*2-1, min(d_max, d3), 1 + 0.000000000000001*bar[0]);
    EmitVertex();

    gl_Position = warp_matrix * vec4(p4/gua_resolution*2-1, min(d_max, d4), 1 + 0.000000000000001*bar[0]);
    EmitVertex();

    EndPrimitive();
  #endif
}

void main() {
  vec2 pos = vec2(vertex_id[0] % gua_resolution.x, vertex_id[0] / gua_resolution.x) + 0.5;
  vec2 tex_coords = pos/vec2(gua_resolution.x, gua_resolution.y);

  color = gua_get_color(tex_coords);
  normal = gua_get_normal(tex_coords);

  emit_primitive(tex_coords);
}

#endif
