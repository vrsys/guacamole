@include "common/header.glsl"

layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec4 gua_in_color;
layout(location=2) in float gua_in_thickness;
layout(location=3) in vec3 gua_in_normal;

@include "common/gua_camera_uniforms.glsl"

@material_uniforms@

@include "common/gua_vertex_shader_output.glsl"

@include "common/gua_global_variable_declaration.glsl"

@material_method_declarations_vert@

void main() {

  @material_input@

  gua_world_position = (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
  gua_view_position  = (gua_model_view_matrix * vec4(gua_in_position, 1.0)).xyz;
  //gua_normal         = (gua_normal_matrix * vec4(gua_in_normal, 0.0)).xyz;
  //gua_tangent        = (gua_normal_matrix * vec4(gua_in_tangent, 0.0)).xyz;
  //gua_bitangent      = (gua_normal_matrix * vec4(gua_in_bitangent, 0.0)).xyz;
  //gua_texcoords      = gua_in_texcoords;
  gua_normal = vec3(1.0, 0.0, 0.0);
  gua_tangent = vec3(0.0, 1.0, 0.0);
  gua_bitangent = vec3(0.0, 0.0, 1.0);
  gua_texcoords = vec2(0.5, 0.5);
  
  gua_metalness      = 0.0;
  gua_roughness      = 1.0;
  gua_emissivity     = 1.0;

  @material_method_calls_vert@

  gua_color          = gua_in_color.xyz;
  gua_alpha          = gua_in_color.a;
  @include "common/gua_varyings_assignment.glsl"


  gl_Position = gua_projection_matrix * vec4(gua_view_position, 1.0);
}
