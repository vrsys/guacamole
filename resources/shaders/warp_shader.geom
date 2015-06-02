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
@include "common/gua_abuffer.glsl"
@include "common/gua_gbuffer_input.glsl"
#define DISPLAY_MODE @display_mode@

uniform mat4 original_projection_view_matrix;

// -----------------------------------------------------------------------------
#if DISPLAY_MODE == 3 // GRID --------------------------------------------------
// -----------------------------------------------------------------------------

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

flat in ivec3 varying_position[];
flat out int cellsize;
out vec2 texcoords;

float gua_get_depth_raw(ivec2 frag_pos) {
    return texelFetch(sampler2D(gua_gbuffer_depth), frag_pos, 0).x * 2.0 - 1.0;
}

void emit_grid_vertex(vec2 position, float depth) {
  texcoords = position / gua_resolution;
  vec4 screen_space_pos = vec4(2.0 * texcoords - 1.0, depth, 1.0);
  vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
  gl_Position = original_projection_view_matrix * vec4(h.xyz / h.w, 1);
  EmitVertex();
}

void main() {
  ivec2 position = varying_position[0].xy;
  float depth = gua_get_depth_raw(position);

  if (depth < 1) {
    emit_grid_vertex(position, depth);

    cellsize = varying_position[0].z;

    #if @debug_mode@ == 1
      const float offset = 0.8;
    #else
      const float offset = 1.0;
    #endif

    position = varying_position[0].xy + ivec2(cellsize-1, 0);
    depth = gua_get_depth_raw(position);
    emit_grid_vertex(position + vec2(offset, 0), depth);

    position = varying_position[0].xy + ivec2(0, cellsize-1);
    depth = gua_get_depth_raw(position);
    emit_grid_vertex(position + vec2(0, offset), depth);

    position = varying_position[0].xy + ivec2(cellsize-1, cellsize-1);
    depth = gua_get_depth_raw(position);
    emit_grid_vertex(position + vec2(offset, offset), depth);

    EndPrimitive();
  }
}


// -----------------------------------------------------------------------------
#else // all other modes -------------------------------------------------------
// -----------------------------------------------------------------------------

layout(points) in;

#if DISPLAY_MODE == 0
  layout(points) out;
  layout(max_vertices = 20) out;
  const bool QUADS = false;
#elif DISPLAY_MODE == 1
  layout(triangle_strip) out;
  layout(max_vertices = 80) out;
  const bool QUADS = true;
#else
  layout(points) out;
  layout(max_vertices = 20) out;
  const bool QUADS = false;
#endif


const int MAX_LAYERS = @warping_max_layers@;

flat in uint vertex_id[];
in float bar[];

out vec3 color;
out vec3 normal;


void emit_primitive(float depth, vec2 frag_pos) {
  if (QUADS) {
    const vec2 half_pixel = vec2(1.0) / vec2(gua_resolution);
    const vec2 offsets[4] = {vec2(half_pixel), vec2(-half_pixel.x, half_pixel.y),
                              vec2(half_pixel.x, -half_pixel.y), vec2(-half_pixel)};

    for (int v=0; v<4; ++v) {
      vec4 screen_space_pos = vec4(frag_pos + offsets[v], depth, 1.0);
      vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
      vec3 position = h.xyz / h.w;

      gl_Position =  original_projection_view_matrix * vec4(position, 1 + 0.000000000000001*bar[0]);

      EmitVertex();
    }

    EndPrimitive();

  } else {
    vec4 screen_space_pos = vec4(frag_pos, depth, 1.0);
    vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
    vec3 position = h.xyz / h.w;


    gl_Position =  original_projection_view_matrix * vec4(position, 1 + 0.000000000000001*bar[0]);

    #if DISPLAY_MODE == 2
      gl_PointSize = 12*(1-gl_Position.z/gl_Position.w);
    #else
      gl_PointSize = 1;
    #endif

    EmitVertex(); EndPrimitive();
  }
}


void main() {

  vec2 pos = vec2(vertex_id[0] % gua_resolution.x, vertex_id[0] / gua_resolution.x) + 0.5;
  vec2 tex_coords = pos/vec2(gua_resolution.x, gua_resolution.y);
  vec2 frag_pos = tex_coords*2-1;

  uint current = vertex_id[0];

  // first layer from gbuffer
  color = gua_get_color(tex_coords);
  normal = gua_get_normal(tex_coords);
  float depth = gua_get_depth(tex_coords);

  if (depth < 1) {
    emit_primitive(depth, frag_pos);
  } else {
    return;
  }


  // skip first abuffer layer
  uvec2 frag = unpackUint2x32(frag_list[current]);
  if (frag.x == 0) {
    return;
  }
  current = frag.x;

  // following layers from abuffer
  for (int i=0; i<MAX_LAYERS-1; ++i) {

    uvec2 frag = unpackUint2x32(frag_list[current]);
    if (frag.x == 0) {
      return;
    }

    current = frag.x;

    uvec4 data = frag_data[current - gua_resolution.x*gua_resolution.y];
    color = vec3(unpackUnorm2x16(data.x), unpackUnorm2x16(data.y).x);
    normal = vec3(unpackSnorm2x16(data.y).y, unpackSnorm2x16(data.z));

    float z = unpack_depth24(frag.y);
    float depth = fma(z, 2.0, -1.0);

    emit_primitive(depth, frag_pos);
  }
}

#endif
