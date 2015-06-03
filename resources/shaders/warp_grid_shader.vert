@include "common/header.glsl"

layout(location=0) in uvec3 position;

flat out uvec3 varying_position;

void main() {
  varying_position = position;
}
