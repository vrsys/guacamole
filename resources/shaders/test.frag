@include "shaders/common/header.glsl"

in vec3 gua_normal;

layout(location=0) out vec3 gua_out_color;
layout(location=1) out vec3 gua_out_normal;

void main() {
  gua_out_color = vec3(1, 1, 0);
  gua_out_normal = gua_normal;
}
