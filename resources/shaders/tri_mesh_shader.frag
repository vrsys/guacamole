@include "common/header.glsl"
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

  if (gua_alpha > @abuf_insertion_threshold@) {
    @include "common/gua_write_gbuffer.glsl"
  }
  else {
    abuf_insert(gl_FragCoord.z);
    discard;
  }
}
