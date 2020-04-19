@include "common/header.glsl"

#if @get_enable_multi_view_rendering@

#if @is_hardware_multi_view_rendering_enabled@
#extension GL_OVR_multiview2: require
layout(num_views = 2) in;
#endif // is_hardware_multi_view_rendering_enabled

#endif //get_enable_multi_view_rendering

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

out vec3 test_color;
void main() {
  


#if @is_hardware_multi_view_rendering_enabled@
  int viewport_index = int(gl_ViewID_OVR);
  test_color = vec3(0.0, 0.0, 1.0);
#elif @get_enable_multi_view_rendering@
  int viewport_index = gl_InstanceID;
  test_color = vec3(0.0, 1.0, 0.0); 
#else
  test_color = vec3(1.0, 0.0, 0.0);
#endif //is_hardware_multi_view_rendering_enabled

  @material_input@



#if @get_enable_multi_view_rendering@
if(0 == viewport_index) {
#endif
  gua_world_position = (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
  gua_view_position  = (gua_model_view_matrix * vec4(gua_in_position, 1.0)).xyz;
#if @get_enable_multi_view_rendering@
} else { // TODO: secondary modelview and normal matrices (see trimeshrenderer.cpp)  #note: 
  gua_world_position = (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
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



#if @get_enable_multi_view_rendering@
if(0 == viewport_index) {
#endif
  gl_Position = gua_view_projection_matrix * vec4(gua_world_position, 1.0);
#if @get_enable_multi_view_rendering@
  } else {
    gl_Position = gua_secondary_view_projection_matrix * vec4(gua_world_position, 1.0);
  }

  gl_ViewportIndex = viewport_index;
#endif
}