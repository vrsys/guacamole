@include "shaders/common/header.glsl"

@include "shaders/common/gua_fragment_shader_input.glsl"

@include "shaders/common/gua_uniforms.glsl"

@material_uniforms

layout(location=0) out vec3 gua_out_color;
layout(location=1) out vec3 gua_out_pbr;
layout(location=2) out vec3 gua_out_normal;

@material_method_declarations

void main() {

@material_method_calls

  gua_out_color     = gua_color;
  gua_out_pbr       = vec3(gua_shinyness);
  gua_out_normal    = gua_normal;
}
