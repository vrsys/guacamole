@include "shaders/common/header.glsl"
@include "shaders/common/gua_fragment_shader_input.glsl"
@include "shaders/common/gua_camera_uniforms.glsl"

@material_uniforms

@include "shaders/common/gua_fragment_shader_output.glsl"
@include "shaders/common/gua_global_variable_declaration.glsl"

@material_method_declarations

void main() {
  // vec3 tmp = gua_varying_color;
  
  @material_input
  @include "shaders/common/gua_global_variable_assignment.glsl"
  @material_method_calls

  // gua_color = tmp;
  
  @include "shaders/common/gua_write_gbuffer.glsl"
}
