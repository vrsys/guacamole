#if @get_enable_hardware_mvr@
@include "common/header_multiview_enabled.glsl"
#else
@include "common/header.glsl"
#endif

#if @get_enable_hardware_mvr@
layout(num_views = 2) in;
#endif

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

  gua_world_position = (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;







#if @get_enable_multi_view_rendering@
if(0 == LAYER_ID) {
#endif
  gua_view_position  = (gua_model_view_matrix * vec4(gua_in_position, 1.0)).xyz;

#if @get_enable_multi_view_rendering@
} else { // TODO: secondary modelview and normal matrices (see trimeshrenderer.cpp)  #note: 
  gua_view_position  = (gua_secondary_model_view_matrix * vec4(gua_in_position, 1.0)).xyz;
}
#endif

  gua_normal         = (gua_normal_matrix * vec4(gua_in_normal, 0.0)).xyz;
  gua_tangent        = (gua_normal_matrix * vec4(gua_in_tangent, 0.0)).xyz;
  gua_bitangent      = (gua_normal_matrix * vec4(gua_in_bitangent, 0.0)).xyz;
  gua_texcoords      = gua_in_texcoords;
  gua_metalness      = 0.01;
  gua_roughness      = 0.1;
  gua_emissivity     = 0;

  @material_method_calls_vert@

  @include "common/gua_varyings_assignment.glsl"

#if @get_enable_hardware_mvr@
  if (0 == LAYER_ID) {
  gl_Position = gua_view_projection_matrix * vec4(gua_world_position, 1.0);
  } else if(1 == gl_ViewID_OVR) {
    gl_Position = gua_secondary_view_projection_matrix * vec4(gua_world_position, 1.0);
  }
#else
  #if @get_enable_multi_view_rendering@
  gl_Layer = LAYER_ID;
  if (0 == LAYER_ID) {
  #endif
    gl_Position = gua_view_projection_matrix * vec4(gua_world_position, 1.0);
  #if @get_enable_multi_view_rendering@
  } else {
    gl_Position = gua_secondary_view_projection_matrix * vec4(gua_world_position, 1.0);
  }
  #endif
#endif

}
