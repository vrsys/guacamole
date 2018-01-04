@include "common/header.glsl"

layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec4 gua_in_color;
layout(location=2) in float gua_in_thickness;
layout(location=3) in vec3 gua_in_normal;

@include "common/gua_camera_uniforms.glsl"

@material_uniforms@

out VertexData {
  vec3  gua_varying_object_position;
  vec4  gua_varying_color_rgba;
  float gua_varying_thickness;
  vec3  gua_varying_rme;
} VertexOut;


@include "common/gua_global_variable_declaration.glsl"
float gua_thickness;

@material_method_declarations_vert@

void main() {

  @material_input@

  //gua_varying_object_position = gua_in_position;
  //gua_world_position = (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
  //gua_view_position  = (gua_model_view_matrix * vec4(gua_in_position, 1.0)).xyz;
  //gua_normal         = (gua_normal_matrix * vec4(gua_in_normal, 0.0)).xyz;
  //gua_tangent        = (gua_normal_matrix * vec4(gua_in_tangent, 0.0)).xyz;
  //gua_bitangent      = (gua_normal_matrix * vec4(gua_in_bitangent, 0.0)).xyz;
  //gua_texcoords      = gua_in_texcoords;

  gua_color      = gua_in_color.rgb;
  gua_alpha      = gua_in_color.a;
  gua_thickness  = gua_in_thickness;
  gua_metalness  = 0.0;
  gua_roughness  = 1.0;
  gua_emissivity = 1.0;


  @material_method_calls_vert@

  VertexOut.gua_varying_object_position = gua_in_position;
  VertexOut.gua_varying_color_rgba      = vec4(gua_color, gua_alpha);
  VertexOut.gua_varying_thickness       = gua_thickness;
  VertexOut.gua_varying_rme             = vec3(gua_metalness, gua_roughness, gua_emissivity);

  //gl_Position = gua_projection_matrix * vec4(gua_view_position, 1.0);
}
