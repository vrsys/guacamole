@include "common/header.glsl"

@include "common/gua_fragment_shader_input.glsl"

uniform float gua_texel_width;
uniform float gua_texel_height;

@include "common/gua_fragment_shader_output.glsl"
@include "common/gua_global_variable_declaration.glsl"

@include "common/gua_abuffer_collect.glsl"


void main()
{
  @material_input@
  @include "common/gua_global_variable_assignment.glsl"

  gua_emissivity = 1.0;
  gua_metalness = 0.0;
  gua_uvs.z = 0.0;
  gua_uvs.w = 0;

  gua_color = vec3(0.01, 0.01, 0.01);

  submit_fragment(gl_FragCoord.z);
}
