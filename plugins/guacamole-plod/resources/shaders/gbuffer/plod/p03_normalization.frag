@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 gua_position_varying;
in vec2 gua_quad_coords;

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
//layout(binding=0) uniform sampler2D p01_depth_texture;
layout(binding=0) uniform sampler2D p02_color_texture;
layout(binding=1) uniform sampler2D p02_normal_texture;
layout(binding=2) uniform sampler2D p02_pbr_texture;
layout(binding=3) uniform sampler2D p01_log_depth_texture;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_fragment_shader_output.glsl"


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {
  vec3  normalized_color  = vec3(1.0);
  float output_depth  = 1.0;
  vec3  output_normal = vec3(0.0);
  vec3 coords = vec3(gua_quad_coords, 0.0);
  vec4 accumulated_color = texture(p02_color_texture, coords.xy);
  

  float accumulated_weight = accumulated_color.a;
  if(accumulated_weight == 0.0)
    discard;
  normalized_color = accumulated_color.rgb / accumulated_weight ;
  //normalized_color = accumulated_color.rgb;
  normalized_color = pow(normalized_color, vec3(1.4));
 
  vec3 accumulated_normal = texture(p02_normal_texture, coords.xy).rgb;
  vec3 normalized_normal = normalize(accumulated_normal.rgb / accumulated_weight);

  //float depthValue = texture2D( p01_depth_texture, coords.xy).r;


  
  vec3 gua_color = normalized_color.rgb;
  //vec3 gua_color = vec3(1.0, 0.0, 0.0);

  vec3 gua_normal = normalized_normal;

  vec3 written_pbr_coeffs = (texture(p02_pbr_texture, coords.xy).rgb) / accumulated_weight;

  float gua_metalness  = written_pbr_coeffs.r;
  float gua_roughness  = written_pbr_coeffs.g;
  float gua_emissivity = written_pbr_coeffs.b;

  bool gua_flags_passthrough = false;
  if(gua_metalness == 0.0 && gua_roughness == 0.0 && gua_emissivity == 0.0)
    gua_flags_passthrough = true;

/////
  {
  @include "common/gua_write_gbuffer.glsl"
  gl_FragDepth = texture(p01_log_depth_texture, coords.xy).r;
  }
}

