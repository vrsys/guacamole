@include "common/header.glsl"

//layout (early_fragment_tests) in;


@include "common/gua_camera_uniforms.glsl"

//uniform float gua_texel_width;
//uniform float gua_texel_height;

in VertexData {
  vec3 gua_varying_world_position;
  vec3 gua_varying_normal;
  vec4 gua_varying_color_rgba;
  vec3 gua_varying_rme;
} VertexIn;


/*
vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}*/

@material_uniforms@

@include "common/gua_fragment_shader_output.glsl"
@include "common/gua_global_variable_declaration.glsl"

@include "common/gua_abuffer_collect.glsl"

@material_method_declarations_frag@

void main() {

  vec3 gua_varying_world_position = VertexIn.gua_varying_world_position;
  vec3 gua_varying_normal = VertexIn.gua_varying_normal;
  vec3 gua_varying_tangent = vec3(1.0, 0.0, 0.0);
  vec3 gua_varying_bitangent =  vec3(0.0, 1.0, 0.0);
  vec2 gua_varying_texcoords =  vec2(0.5, 0.5);
  vec3 gua_varying_color =  VertexIn.gua_varying_color_rgba.rgb;

  float gua_varying_roughness  =  VertexIn.gua_varying_rme.x;
  float gua_varying_metalness  =  VertexIn.gua_varying_rme.y;
  float gua_varying_emissivity =  VertexIn.gua_varying_rme.z;

  @material_input@
  @include "common/gua_global_variable_assignment.glsl"

  float gua_alpha = VertexIn.gua_varying_color_rgba.a;

  
  // normal mode or high fidelity shadows
  if (gua_rendering_mode != 1) {
    @material_method_calls_frag@
  }
  gua_color = gua_varying_color;
  gua_emissivity = 1.0;
  //gua_color = vec3(1.0, 0.0, 0.0);
  //gua_alpha = 0.2;

  submit_fragment(gl_FragCoord.z);
}
