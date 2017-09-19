@include "common/header.glsl"

layout(location=0) in int in_quantized_pos_x;
layout(location=1) in int in_quantized_pos_y;
layout(location=2) in int in_quantized_pos_z;
layout(location=3) in int in_short_padding_0;
layout(location=4) in float in_col_r;
layout(location=5) in float in_col_g;
layout(location=6) in float in_col_b;
layout(location=7) in float in_char_padding_0;


uniform vec3 bounding_box_min = vec3(0.0, 0.0, 0.0);
uniform float voxel_thickness  = 0.0;

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


  gua_color      = vec3(in_col_r, in_col_g, in_col_b);
  gua_alpha      = 1.0;

  gua_metalness  = 0.0;
  gua_roughness  = 1.0;
  gua_emissivity = 1.0;


  @material_method_calls_vert@

  vec3 quantized_pos = vec3(in_quantized_pos_x,
                            in_quantized_pos_y,
                            in_quantized_pos_z);

  vec3 dequantized_object_space_position 
    = voxel_thickness * quantized_pos + bounding_box_min;

  gua_thickness  = voxel_thickness / 2.0;

  VertexOut.gua_varying_object_position = dequantized_object_space_position;
  VertexOut.gua_varying_color_rgba      = vec4(gua_color, gua_alpha);
  VertexOut.gua_varying_rme             = vec3(gua_metalness, gua_roughness, gua_emissivity);
  VertexOut.gua_varying_thickness       = gua_thickness;
  //gl_Position = gua_projection_matrix * vec4(gua_view_position, 1.0);
}
