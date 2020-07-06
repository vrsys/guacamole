@include "common/header.glsl"

// input
layout(location=0) in vec3 gua_in_position;

// uniforms
@include "common/gua_camera_uniforms.glsl"

// 4 float layout
layout (std140, binding=1) uniform lightBlock {
  mat4  light_mvp_matrices[32];
#if @get_enable_multi_view_rendering@
  mat4  secondary_light_mvp_matrices[32];
#endif
};


uniform uint light_type_offset;

// varyings
out flat uint light_id_shift_5;
out flat uint light_bit;

#if @get_enable_multi_view_rendering@
out flat uint light_table_id;
#endif

void main() {
  uint base_light_id = gl_InstanceID + light_type_offset;
#if @get_enable_multi_view_rendering@
  light_table_id = (base_light_id % 2);

  base_light_id /= 2; //account for duplication of num IDs
#endif

#if @get_enable_multi_view_rendering@
  if(1 == light_table_id) {
  	gl_Position = light_mvp_matrices[base_light_id] * vec4(gua_in_position, 1.0);
  } else {
#endif
  gl_Position = secondary_light_mvp_matrices[base_light_id] * vec4(gua_in_position, 1.0);
#if @get_enable_multi_view_rendering@
  }
#endif
  light_id_shift_5 = base_light_id >> 5;
  light_bit = 1u << (base_light_id % 32);
}
