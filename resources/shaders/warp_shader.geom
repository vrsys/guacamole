@include "common/header.glsl"

#define DISPLAY_MODE @display_mode@

layout(points) in;

#if DISPLAY_MODE == 0
layout(points) out;
layout(max_vertices = 20) out;
const bool QUADS = false;
#else
layout(triangle_strip) out;
layout(max_vertices = 80) out;
const bool QUADS = true;
#endif

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_abuffer.glsl"

uniform mat4 original_inverse_projection_view_matrix;

const int MAX_LAYERS = @warping_max_layers@;

flat in uint vertex_id[];
in float bar[];

flat out uint data_pos;

void main() {

  vec2 pos = vec2(vertex_id[0] % gua_resolution.x, vertex_id[0] / gua_resolution.x);
  vec2 frag_pos = pos/vec2(gua_resolution.x, gua_resolution.y)*2-1;

  uint current = vertex_id[0];

  for (int i=0; i<MAX_LAYERS; ++i) {

    uvec2 frag = unpackUint2x32(frag_list[current]);
    if (frag.x == 0) {
      return;
    }

    current = frag.x;

    float z = unpack_depth24(frag.y);
    float depth = fma(z, 2.0, -1.0);

    if (QUADS) {

      const vec2 half_pixel = vec2(1.0) / vec2(gua_resolution);
      const vec2 offsets[4] = {vec2(half_pixel), vec2(-half_pixel.x, half_pixel.y),
                                vec2(half_pixel.x, -half_pixel.y), vec2(-half_pixel)};

      data_pos = current - gua_resolution.x*gua_resolution.y;

      for (int v=0; v<4; ++v) {
        vec4 screen_space_pos = vec4(frag_pos + offsets[v], depth, 1.0);
        vec4 h = original_inverse_projection_view_matrix * screen_space_pos;
        vec3 position = h.xyz / h.w;

        gl_Position =  gua_projection_matrix * gua_view_matrix * vec4(position, 1 + 0.000000000000001*bar[0]);

        EmitVertex();
      }
      
      EndPrimitive();

    } else {
      vec4 screen_space_pos = vec4(frag_pos, depth, 1.0);
      vec4 h = original_inverse_projection_view_matrix * screen_space_pos;
      vec3 position = h.xyz / h.w;

      data_pos = current - gua_resolution.x*gua_resolution.y;
      gl_PointSize = 1;
      gl_Position =  gua_projection_matrix * gua_view_matrix * vec4(position, 1 + 0.000000000000001*bar[0]);

      EmitVertex(); EndPrimitive();
    }
  }
}