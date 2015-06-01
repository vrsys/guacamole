@include "common/header.glsl"

@include "shaders/common/gua_camera_uniforms.glsl"

flat in int cellsize;

// output
layout(location=0) out vec3 gua_out_color;

void main() {
  float intensity = log2(cellsize) / 5.0;
  gua_out_color = vec3(0.5 * (1-intensity), 0.5 * intensity, 0.1);
}