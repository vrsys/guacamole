@include "common/header.glsl"

@include "common/gua_fragment_shader_input.glsl"

@include "common/gua_fragment_shader_output.glsl"

@include "common/gua_global_variable_declaration.glsl"


// caution: only use this shader if the final pass is supposed to be the color debug view pass
void main()
{
  //gua_out_color = 3 * vec3(0.005, 0.01, 0.02);
  gua_out_color =  vec3(0.0, 0.0, 0.2);
}
