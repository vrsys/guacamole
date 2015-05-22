@include "common/header.glsl"

layout(location=0) in float foo;

flat out uint vertex_id;

out float bar;

void main() {
  vertex_id = gl_VertexID;
  bar = foo;
}
