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
@include "hole_filling_modes.glsl"


uniform mat4 warp_matrix;

float get_depth_raw(vec2 frag_pos) {
  return texelFetch(sampler2D(gua_gbuffer_depth), ivec2(frag_pos), 0).x*2-1;
}

float get_min_depth(vec2 frag_pos) {
  const float depth0 = get_depth_raw(frag_pos + vec2(-0.5, -0.5));
  const float depth1 = get_depth_raw(frag_pos + vec2( 0.5, -0.5));
  const float depth2 = get_depth_raw(frag_pos + vec2(-0.5,  0.5));
  const float depth3 = get_depth_raw(frag_pos + vec2( 0.5,  0.5));

  return min(depth0, min(depth1, min(depth2, depth3)));
}

float get_depth(vec2 position) {
  #if HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_1 || HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_2
    return get_min_depth(position);
  #elif WARP_MODE == WARP_MODE_GRID_ADVANCED_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION
    return gua_get_depth(position/gua_resolution);
  #else
    return get_depth_raw(position);
  #endif
}

void emit_grid_vertex(vec2 position, float depth) {
  gl_Position = warp_matrix * vec4(2.0 * (position / gua_resolution) - 1.0, depth, 1.0);
  EmitVertex();
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
#if HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_1 || HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_2
  flat out uint is_rubber_band;
#endif

out vec2 texcoords;



void emit_quad(uvec2 offset, uvec2 size) {

  if (size.x > 0 && size.y > 0) {

    float depth1, depth2, depth3, depth4;

    #if HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_1 || HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_2
      const float interpolation_offset = 0;
    #else
      const float interpolation_offset = 0.5;
    #endif

    vec2 pos1 = vec2(varying_position[0].xy)                        + vec2(offset);
    vec2 pos2 = vec2(varying_position[0].xy) + vec2(size.x, 0)      + vec2(offset);
    vec2 pos3 = vec2(varying_position[0].xy) + vec2(0,      size.y) + vec2(offset);
    vec2 pos4 = vec2(varying_position[0].xy) + vec2(size.x, size.y) + vec2(offset);

    #if WARP_MODE == WARP_MODE_GRID_DEPTH_THRESHOLD || WARP_MODE == WARP_MODE_GRID_SURFACE_ESTIMATION

      depth1 = get_depth(pos1+vec2( interpolation_offset,  interpolation_offset));
      depth2 = get_depth(pos2+vec2(-interpolation_offset,  interpolation_offset));
      depth3 = get_depth(pos3+vec2( interpolation_offset, -interpolation_offset));
      depth4 = get_depth(pos4+vec2(-interpolation_offset, -interpolation_offset));

    #elif WARP_MODE == WARP_MODE_GRID_ADVANCED_SURFACE_ESTIMATION || WARP_MODE == WARP_MODE_GRID_NON_UNIFORM_SURFACE_ESTIMATION

      const int cont_l = int(varying_position[0].z >> BIT_CONTINUOUS_L) & 1;
      const int cont_r = int(varying_position[0].z >> BIT_CONTINUOUS_R) & 1;
      const int cont_t = int(varying_position[0].z >> BIT_CONTINUOUS_T) & 1;
      const int cont_b = int(varying_position[0].z >> BIT_CONTINUOUS_B) & 1;

      const int cont_tl = int(varying_position[0].z >> BIT_CONTINUOUS_TL) & 1;
      const int cont_tr = int(varying_position[0].z >> BIT_CONTINUOUS_TR) & 1;
      const int cont_bl = int(varying_position[0].z >> BIT_CONTINUOUS_BL) & 1;
      const int cont_br = int(varying_position[0].z >> BIT_CONTINUOUS_BR) & 1;

      depth1 = get_depth(vec2(-cont_l, -cont_b)*cont_bl*interpolation_offset + pos1+vec2( interpolation_offset,  interpolation_offset));
      depth2 = get_depth(vec2( cont_r, -cont_b)*cont_br*interpolation_offset + pos2+vec2(-interpolation_offset,  interpolation_offset));
      depth3 = get_depth(vec2(-cont_l,  cont_t)*cont_tl*interpolation_offset + pos3+vec2( interpolation_offset, -interpolation_offset));
      depth4 = get_depth(vec2( cont_r,  cont_t)*cont_tr*interpolation_offset + pos4+vec2(-interpolation_offset, -interpolation_offset));

    #endif // ------------------------------------------------------------------

    cellsize = min(size.x, size.y);

    #if HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_1 || HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_2
      const float max_depth = max(depth1, max(depth2, max(depth3, depth4)));
      const float min_depth = min(depth1, min(depth2, min(depth3, depth4)));
      if (max_depth - min_depth > @rubber_band_threshold@ && size.x == 1 && size.y == 1) {
        is_rubber_band = 1;
      } else {
        is_rubber_band = 0;
      }
    #endif

    texcoords = pos1 / gua_resolution;
    emit_grid_vertex(pos1 + vec2(-GAP, -GAP), depth1);

    texcoords = pos2 / gua_resolution;
    emit_grid_vertex(pos2 + vec2( GAP, -GAP), depth2);

    texcoords = pos3 / gua_resolution;
    emit_grid_vertex(pos3 + vec2(-GAP,  GAP), depth3);

    texcoords = pos4 / gua_resolution;
    emit_grid_vertex(pos4 + vec2( GAP,  GAP), depth4);

    EndPrimitive();
  }
}

void emit_pixel(uvec2 offset) {

  #if HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_1 || HOLE_FILLING_MODE == HOLE_FILLING_RUBBER_BAND_2
    emit_quad(offset, uvec2(1));

  #else
    cellsize = 1;
    vec2 position = varying_position[0].xy + offset;
    texcoords = position / gua_resolution;
    const float depth = get_depth_raw(position);

    emit_grid_vertex(position + vec2(0, 0) + vec2(-GAP, -GAP), depth);
    emit_grid_vertex(position + vec2(1, 0) + vec2( GAP, -GAP), depth);
    emit_grid_vertex(position + vec2(0, 1) + vec2(-GAP,  GAP), depth);
    emit_grid_vertex(position + vec2(1, 1) + vec2( GAP,  GAP), depth);

    EndPrimitive();
  #endif
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

  if (quad2.z > 0) {
    emit_pixel(quad2.xy);
  }

  if (quad3.z > 0) {
    emit_pixel(quad3.xy);
  }

#else

  if ((varying_position[0].z & 1) > 0) {
    emit_quad(uvec2(0), uvec2(1 << (varying_position[0].z >> BIT_CURRENT_LEVEL)));
  } else {
    // emit_quad(uvec2(0), uvec2(2));
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

void main() {
  vec2 pos = vec2(vertex_id[0] % gua_resolution.x, vertex_id[0] / gua_resolution.x) + 0.5;
  vec2 tex_coords = pos/vec2(gua_resolution.x, gua_resolution.y);

  color = gua_get_color(tex_coords);
  normal = gua_get_normal(tex_coords);

  float depth = gua_get_depth(tex_coords);
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

  #elif WARP_MODE == WARP_MODE_QUADS_DEPTH_ALIGNED

    vec2 p = pos + vec2(-0.5, -0.5);
    emit_grid_vertex(p, get_min_depth(p));

    p = pos + vec2( 0.5, -0.5);
    emit_grid_vertex(p, get_min_depth(p));

    p = pos + vec2(-0.5,  0.5);
    emit_grid_vertex(p, get_min_depth(p));

    p = pos + vec2( 0.5,  0.5);
    emit_grid_vertex(p, get_min_depth(p));

    color = gua_get_color(tex_coords);
    normal = gua_get_normal(tex_coords);

    EndPrimitive();
  #endif
}

#endif
