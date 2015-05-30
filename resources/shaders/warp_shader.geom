@include "common/header.glsl"

#define DISPLAY_MODE @display_mode@

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

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_abuffer.glsl"
@include "common/gua_gbuffer_input.glsl"

uniform mat4 original_inverse_projection_view_matrix;

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


    gl_Position =  gua_projection_matrix * gua_view_matrix * vec4(position, 1 + 0.000000000000001*bar[0]);

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