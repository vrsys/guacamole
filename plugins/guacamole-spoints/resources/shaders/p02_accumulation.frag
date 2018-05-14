@include "shaders/common/header.glsl"
@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/pack_vec3.glsl"



in vec3 pass_point_color;

layout (location = 0) out vec3 out_accumulated_color;
layout (location = 3) out vec2 out_accumulated_weight_and_depth;

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  vec2 centered_point_coord = (gl_PointCoord.xy - 0.5) * 2.0;
  float blend_weight = 2.0 - dot(centered_point_coord, centered_point_coord) / 2.0;


  out_accumulated_color = vec3(blend_weight * pass_point_color);

  out_accumulated_weight_and_depth = vec2(blend_weight, blend_weight * gl_FragCoord.z);

}
