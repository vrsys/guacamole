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

// -----------------------------------------------------------------------------
#if WARP_MODE == WARP_MODE_GRID || WARP_MODE == WARP_MODE_ADAPTIVE_GRID // -----
// -----------------------------------------------------------------------------

#if @debug_cell_gap@ == 1
  #define GAP 0.6
#else
  #define GAP 1.0
#endif

layout(points) in;
layout(triangle_strip, max_vertices = 16) out;

@include "shaders/warp_grid_bits.glsl"

flat in  uvec3 varying_position[];
flat out uint cellsize;

out vec2 texcoords;

float gua_get_depth_raw(vec2 frag_pos) {
  return texelFetch(sampler2D(gua_gbuffer_depth), ivec2(frag_pos), 0).x * 2.0 - 1.0;
}

void emit_grid_vertex(vec2 position, float depth) {
  texcoords = position / gua_resolution;
  gl_Position = warp_matrix * vec4(2.0 * texcoords - 1.0, depth, 1.0);
  EmitVertex();
}

void emit_quad(uvec2 offset, uint size) {
  
  vec2 position = varying_position[0].xy+offset;
  float depth = gua_get_depth_raw(position);
  
  if (depth < 1) {

    cellsize = size;

    #if WARP_MODE == WARP_MODE_GRID // -----------------------------------------
      emit_grid_vertex(position + vec2(0, 0), depth);

      position = varying_position[0].xy+offset + vec2(cellsize-1, 0);
      depth = gua_get_depth_raw(position);
      emit_grid_vertex(position + vec2(GAP, 0), depth);

      position = varying_position[0].xy+offset + vec2(0, cellsize-1);
      depth = gua_get_depth_raw(position);
      emit_grid_vertex(position + vec2(0, GAP), depth);

      position = varying_position[0].xy+offset + vec2(cellsize-1, cellsize-1);
      depth = gua_get_depth_raw(position);
      emit_grid_vertex(position + vec2(GAP, GAP), depth);

    #elif WARP_MODE == WARP_MODE_ADAPTIVE_GRID // ------------------------------

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
      depth = gua_get_depth( (position + 0.5*(1+lookup_offset)) / gua_resolution);
      emit_grid_vertex(position + vec2(0, 0), depth);

      position = varying_position[0].xy+offset + vec2(cellsize-1, 0);
      lookup_offset = vec2(cont_r, -cont_b) * cont_br;
      depth = gua_get_depth( (position + 0.5*(1+lookup_offset)) / gua_resolution);
      emit_grid_vertex(position + vec2(GAP, 0), depth);

      position = varying_position[0].xy+offset + vec2(0, cellsize-1);
      lookup_offset = vec2(-cont_l, cont_t) * cont_tl;
      depth = gua_get_depth( (position + 0.5*(1+lookup_offset)) / gua_resolution);
      emit_grid_vertex(position + vec2(0, GAP), depth);

      position = varying_position[0].xy+offset + vec2(cellsize-1, cellsize-1);
      lookup_offset = vec2(cont_r, cont_t) * cont_tr;
      depth = gua_get_depth( (position + 0.5*(1+lookup_offset)) / gua_resolution);
      emit_grid_vertex(position + vec2(GAP, GAP), depth);

    #endif // ------------------------------------------------------------------

    EndPrimitive();
  }
}

void main() {

  if ((varying_position[0].z & 1) > 0) {
    emit_quad(uvec2(0), 1 << (varying_position[0].z >> BIT_CURRENT_LEVEL));
  } else {
    const uvec2 offsets[4] = {uvec2(0), uvec2(1, 0),
                              uvec2(1), uvec2(0, 1)};
    for (int v=0; v<4; ++v) {
      emit_quad(offsets[v], 1);
    }
  }
}


// -----------------------------------------------------------------------------
#else // all other modes -------------------------------------------------------
// -----------------------------------------------------------------------------

layout(points) in;

#if WARP_MODE == WARP_MODE_POINTS || WARP_MODE == WARP_MODE_SCALED_POINTS
  layout(points) out;
  layout(max_vertices = 1) out;
#else
  layout(triangle_strip) out;
  layout(max_vertices = 4) out;
#endif


const int MAX_LAYERS = @warping_max_layers@;

flat in uint vertex_id[];
in float bar[];

out vec3 color;
out vec3 normal;


void emit_primitive(float depth, vec2 frag_pos) {
  #if WARP_MODE == WARP_MODE_QUADS
    const vec2 half_pixel = vec2(1.0) / vec2(gua_resolution);
    const vec2 offsets[4] = {vec2(half_pixel), vec2(-half_pixel.x, half_pixel.y),
                              vec2(half_pixel.x, -half_pixel.y), vec2(-half_pixel)};

    for (int v=0; v<4; ++v) {
      vec3 screen_space_pos = vec3(frag_pos + offsets[v], depth);
      gl_Position = warp_matrix * vec4(screen_space_pos, 1 + 0.000000000000001*bar[0]);

      EmitVertex();
    }

    EndPrimitive();

  #else
    vec3 screen_space_pos = vec3(frag_pos, depth);
    gl_Position = warp_matrix * vec4(screen_space_pos, 1 + 0.000000000000001*bar[0]);

    #if WARP_MODE == WARP_MODE_SCALED_POINTS
      gl_PointSize = 12*(1-gl_Position.z/gl_Position.w);
    #else
      gl_PointSize = 1;
    #endif

    EmitVertex(); EndPrimitive();
  #endif
}

void main() {

  vec2 pos = vec2(vertex_id[0] % gua_resolution.x, vertex_id[0] / gua_resolution.x) + 0.5;
  vec2 tex_coords = pos/vec2(gua_resolution.x, gua_resolution.y);
  vec2 frag_pos = tex_coords*2-1;

  uint current = vertex_id[0];

  color = gua_get_color(tex_coords);
  normal = gua_get_normal(tex_coords);
  float depth = gua_get_depth(tex_coords);

  if (depth < 1) {
    emit_primitive(depth, frag_pos);
  }
}

#endif
