@include "common/header.glsl"

layout(points) in;
layout(points) out;
layout(max_vertices = 10) out;

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_abuffer.glsl"

uniform mat4 warp_matrix1;
uniform mat4 warp_matrix2;


flat in uint vertex_id[];
in float bar[];

flat out uint data_pos;

void main() {

  vec2 pos = vec2(vertex_id[0] % 1920, vertex_id[0] / 1920);
  vec2 frag_pos = pos/vec2(1920, 1080)*2-1;

  int frag_count = 0;
  uint current = vertex_id[0];

  while (frag_count < ABUF_MAX_FRAGMENTS) {

    uvec2 frag = unpackUint2x32(frag_list[current]);
    current = frag.x;
    if (current == 0) {
      break;
    }

    ++frag_count;

    float z = unpack_depth24(frag.y);
    float depth = fma(z, 2.0, -1.0);

    vec4 screen_space_pos = vec4(frag_pos, depth, 1.0);
    vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
    vec3 position = h.xyz / h.w;

    data_pos = current - 1920*1080;
    gl_PointSize = 1;

    gl_Position = gua_projection_matrix * gua_view_matrix * warp_matrix1 * vec4(position, 1 + 0.000001*bar[0]);
    gl_Position /= gl_Position.w;
    gl_Position *= vec4(0.5, 1, 1, 1);
    gl_Position -= vec4(0.5, 0, 0, 0);
    EmitVertex(); EndPrimitive();

    gl_Position = gua_projection_matrix * gua_view_matrix * warp_matrix2 * vec4(position, 1 + 0.000001*bar[0]);
    gl_Position /= gl_Position.w;
    gl_Position *= vec4(0.5, 1, 1, 1);
    gl_Position += vec4(0.5, 0, 0, 0);

    EmitVertex(); EndPrimitive();

    break;

  }
}