@include "common/header.glsl"

layout(location=0) in ivec3 position;

flat out ivec3 varying_position;

void main() {
  varying_position = position;
}
