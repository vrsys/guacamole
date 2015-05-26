@include "common/header.glsl"

layout(points) in;
layout(points) out;
layout(max_vertices = 10) out;

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_abuffer.glsl"

uniform mat4 original_inverse_projection_view_matrix;


flat in uint vertex_id[];
in float bar[];

flat out uint data_pos;

void main() {

  vec2 pos = vec2(vertex_id[0] % gua_resolution.x, vertex_id[0] / gua_resolution.x);
  vec2 frag_pos = pos/vec2(gua_resolution.x, gua_resolution.y)*2-1;

  uint current = vertex_id[0];

  // const int MAX_LAYERS = 1;
  const int MAX_LAYERS = ABUF_MAX_FRAGMENTS;

  for (int i=0; i<MAX_LAYERS; ++i) {

    uvec2 frag = unpackUint2x32(frag_list[current]);
    if (frag.x == 0) {
      return;
    }

    current = frag.x;

    float z = unpack_depth24(frag.y);
    float depth = fma(z, 2.0, -1.0);

    vec4 screen_space_pos = vec4(frag_pos, depth, 1.0);
    vec4 h = original_inverse_projection_view_matrix * screen_space_pos;
    vec3 position = h.xyz / h.w;

    data_pos = current - gua_resolution.x*gua_resolution.y;
    gl_PointSize = 1;
    gl_Position =  gua_projection_matrix * gua_view_matrix * vec4(position, 1 + 0.000000000000001*bar[0]);

    EmitVertex(); EndPrimitive();
  }
}