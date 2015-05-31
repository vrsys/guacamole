@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_abuffer.glsl"

in vec3 color;
in vec3 normal;

// output
layout(location=0) out vec3 gua_out_color;

void main() {
  gua_out_color = color;
}