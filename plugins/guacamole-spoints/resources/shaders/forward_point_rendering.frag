@include "shaders/common/header.glsl"
@include "shaders/common/gua_fragment_shader_input.glsl"
@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/pack_vec3.glsl"

@material_uniforms@

///////////////////////////////////////////////////////////////////////////////
// video3D uniforms
///////////////////////////////////////////////////////////////////////////////


@include "shaders/common/gua_fragment_shader_output.glsl"
@include "shaders/common/gua_global_variable_declaration.glsl"

@material_method_declarations_frag@
@include "common/gua_abuffer_collect.glsl"

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  @material_input@

  @include "shaders/common/gua_global_variable_assignment.glsl"

  gua_normal = vec3(1.0, 0.0, 0.0);
  //gua_color = vec3(1.0, 0.0, 0.0);
  gua_emissivity = 1.0;

  vec2 centered_point_coord = (gl_PointCoord.xy - 0.5) * 2.0;
  float blend_weight = 2.0 - dot(centered_point_coord, centered_point_coord) / 2.0;
  gua_color = mix(vec3(0.0, 0.0, 0.0), gua_color, blend_weight);
  //@material_method_calls_frag@

  submit_fragment(gl_FragCoord.z);
}
