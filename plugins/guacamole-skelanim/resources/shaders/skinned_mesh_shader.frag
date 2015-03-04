@include "common/header.glsl"

//layout (early_fragment_tests) in;

@include "common/gua_fragment_shader_input.glsl"
@include "common/gua_camera_uniforms.glsl"

@material_uniforms@

@include "common/gua_fragment_shader_output.glsl"
@include "common/gua_global_variable_declaration.glsl"

@include "common/gua_abuffer_collect.glsl"

@material_method_declarations_frag@

void main() {
  @material_input@
  @include "common/gua_global_variable_assignment.glsl"
  @material_method_calls_frag@

  submit_fragment(gl_FragCoord.z);
}