@include "common/header.glsl"

layout (location = 0) in ivec3 in_position;  

out ivec3 varying_position;

void main() {
  varying_position = in_position;
}
