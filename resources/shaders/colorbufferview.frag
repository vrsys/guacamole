@include "common/header.glsl"

// varyings
in vec2 gua_quad_coords;

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"
@include "common/gua_shading.glsl"


// output
layout(location=0) out vec3 gua_out_color;

void main() {
  gua_out_color = gua_get_color(gua_quad_coords);
}

