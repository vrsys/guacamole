@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
layout(location=0) in vec3 gua_in_position;
layout(location=2) in vec2 gua_in_texcoord;

@include "shaders/common/gua_camera_uniforms.glsl"

@material_uniforms@

@include "shaders/common/gua_vertex_shader_output.glsl"
@include "shaders/common/gua_global_variable_declaration.glsl"

@material_method_declarations_vert@

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  gua_world_position = vec3(0);
  gua_normal     = vec3(0);
  gua_tangent    = vec3(0);
  gua_bitangent  = vec3(0);
  gua_texcoords  = gua_in_texcoord;
  gua_metalness  = 0.5;
  gua_roughness  = 0.5;
  gua_emissivity = 1;

  @material_method_calls_vert@

  @include "shaders/common/gua_varyings_assignment.glsl"

  gl_Position = vec4(gua_in_position, 1.0);
}
