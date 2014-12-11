@include "shaders/common/header.glsl"
@include "shaders/common/gua_fragment_shader_input.glsl"
@include "shaders/common/gua_camera_uniforms.glsl"

@material_uniforms

@include "shaders/common/gua_fragment_shader_output.glsl"
@include "shaders/common/gua_global_variable_declaration.glsl"

@include "shaders/common/gua_abuffer.glsl"
@include "shaders/common/gua_abuffer_collect.glsl"

@material_method_declarations

void main() {
  @material_input
  @include "shaders/common/gua_global_variable_assignment.glsl"
  @material_method_calls

  if (gua_alpha > 0.999) {

  @include "shaders/common/gua_write_gbuffer.glsl"

  }
  else {
    abuf_insert();
    discard;
  }
}
