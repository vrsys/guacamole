@include "common/header.glsl"

layout(location=0) in int in_quantized_pos_x;
layout(location=1) in int in_quantized_pos_y;
layout(location=2) in int in_quantized_pos_z;
layout(location=3) in int in_short_padding_0;
layout(location=4) in float in_col_r;
layout(location=5) in float in_col_g;
layout(location=6) in float in_col_b;
layout(location=7) in float in_char_padding_0;


uniform vec3 bounding_box_min = vec3(-1.0, 0.03, -1.0);
uniform vec3 bounding_box_max = vec3(0.0, 0.0, 0.0);
uniform float voxelsize_sent  = 0.008;


@include "common/gua_camera_uniforms.glsl"

@material_uniforms@

@include "common/gua_vertex_shader_output.glsl"

@include "common/gua_global_variable_declaration.glsl"

@material_method_declarations_vert@

void main() {

  @material_input@

  vec3 quantized_pos = vec3(in_quantized_pos_x,
                            in_quantized_pos_y,
                            in_quantized_pos_z);

  vec3 in_color = vec3(in_col_r, in_col_g, in_col_b);

  vec3 dequantized_object_space_position 
    = voxelsize_sent * quantized_pos + bounding_box_min;


  gua_world_position = (gua_model_matrix * vec4(dequantized_object_space_position, 1.0)).xyz;
  gua_view_position  = (gua_model_view_matrix * vec4(dequantized_object_space_position, 1.0)).xyz;
  //gua_normal         = (gua_normal_matrix * vec4(gua_in_normal, 0.0)).xyz;
  //gua_tangent        = (gua_normal_matrix * vec4(gua_in_tangent, 0.0)).xyz;
  //gua_bitangent      = (gua_normal_matrix * vec4(gua_in_bitangent, 0.0)).xyz;
  //gua_texcoords      = gua_in_texcoords;
  gua_metalness      = 0.01;
  gua_roughness      = 0.1;
  gua_emissivity     = 1.0;

  @material_method_calls_vert@
  gua_color          = in_color;
  gua_alpha          = 1.0;
  @include "common/gua_varyings_assignment.glsl"


  gl_Position = gua_projection_matrix * vec4(gua_view_position, 1.0);
}