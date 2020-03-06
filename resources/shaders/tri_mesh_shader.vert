@include "common/header.glsl"

layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;

@include "common/gua_camera_uniforms.glsl"

@material_uniforms@

@include "common/gua_vertex_shader_output.glsl"

@include "common/gua_global_variable_declaration.glsl"

@material_method_declarations_vert@

void main() {

  @material_input@

if(0 == gl_InstanceID) {
  gua_world_position = (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
  gua_view_position  = (gua_model_view_matrix * vec4(gua_in_position, 1.0)).xyz;
  gua_normal         = (gua_normal_matrix * vec4(gua_in_normal, 0.0)).xyz;
  gua_tangent        = (gua_normal_matrix * vec4(gua_in_tangent, 0.0)).xyz;
  gua_bitangent      = (gua_normal_matrix * vec4(gua_in_bitangent, 0.0)).xyz;
  gua_texcoords      = gua_in_texcoords;
  gua_metalness      = 0.01;
  gua_roughness      = 0.1;
  gua_emissivity     = 0;
} else { // TODO: secondary modelview and normal matrices (see trimeshrenderer.cpp)
  gua_world_position = (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
  gua_view_position  = (gua_model_view_matrix * vec4(gua_in_position, 1.0)).xyz;
  gua_normal         = (gua_normal_matrix * vec4(gua_in_normal, 0.0)).xyz;
  gua_tangent        = (gua_normal_matrix * vec4(gua_in_tangent, 0.0)).xyz;
  gua_bitangent      = (gua_normal_matrix * vec4(gua_in_bitangent, 0.0)).xyz;
  gua_texcoords      = gua_in_texcoords;
  gua_metalness      = 0.01;
  gua_roughness      = 0.1;
  gua_emissivity     = 0;
}

  @material_method_calls_vert@

  @include "common/gua_varyings_assignment.glsl"

#if @get_enable_multi_view_rendering@
if (0 == gl_InstanceID) {
#endif
  
  gl_Position = gua_view_projection_matrix * vec4(gua_world_position, 1.0);

#if @get_enable_multi_view_rendering@
} else {
  gl_Position = gua_secondary_view_projection_matrix * vec4(gua_world_position, 1.0);
}

  gl_ViewportIndex = gl_InstanceID;
#endif
}
