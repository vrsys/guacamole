@include "common/header.glsl"

layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;

@include "common/gua_camera_uniforms.glsl"

@include "common/gua_vertex_shader_output.glsl"

@include "common/gua_global_variable_declaration.glsl"


void main() {
  gua_world_position = (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
  gua_view_position  = (gua_model_view_matrix * vec4(gua_in_position, 1.0)).xyz;
  gua_normal         = (gua_normal_matrix * vec4(gua_in_normal, 0.0)).xyz;
  gua_tangent        = (gua_normal_matrix * vec4(gua_in_tangent, 0.0)).xyz;
  gua_bitangent      = (gua_normal_matrix * vec4(gua_in_bitangent, 0.0)).xyz;
  gua_texcoords      = gua_in_texcoords;
  gua_metalness      = 0.01;
  gua_roughness      = 0.1;
  gua_emissivity     = 0;

  @include "common/gua_varyings_assignment.glsl"

  gl_Position = gua_projection_matrix * vec4(gua_view_position, 1.0);

  //gl_ViewportIndex = 1;
}
