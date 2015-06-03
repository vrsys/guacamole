@include "common/header.glsl"

layout (location = 0) in uvec3 in_position;  

out uvec3 varying_position;

void main() {
  varying_position = in_position;
}
