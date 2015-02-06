@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 gua_position_varying;
in vec2 gua_quad_coords;

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
uniform int   using_default_pbr_material;
layout(binding=0) uniform sampler2D p02_color_texture;
layout(binding=1) uniform sampler2D p02_normal_texture;


///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
layout (location=0) out vec3 out_normalized_color;
layout (location=1) out vec3 out_normalized_normal;

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  vec3  normalized_color  = vec3(1.0);
  float output_depth  = 1.0;
  vec3  output_normal = vec3(0.0);
  vec3 coords = vec3(gua_quad_coords, 0.0);
  vec4 accumulated_color = texture(p02_color_texture, coords.xy);
  vec3 accumulated_normal = texture(p02_normal_texture, coords.xy).rgb;

  float accumulated_weight = accumulated_color.a;
  normalized_color = accumulated_color.rgb / accumulated_weight ;
  //normalized_color = accumulated_color.rgb;
  normalized_color = pow(normalized_color, vec3(1.4));
 
  vec3 normalized_normal = normalize(accumulated_normal.rgb / accumulated_weight);

  out_normalized_color = normalized_color;
  out_normalized_normal = normalized_normal;
}

