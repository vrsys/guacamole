@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
//in vec3 pass_color;
in vec2 pass_uvs;
///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_fragment_shader_input.glsl"
@include "common/gua_camera_uniforms.glsl"
///////////////////////////////////////////////////////////////////////////////

layout(binding=0) uniform sampler2D color_texture_atlas;
@material_uniforms@

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_fragment_shader_output.glsl"
@include "common/gua_global_variable_declaration.glsl"

@include "common/gua_abuffer_collect.glsl"

@material_method_declarations_frag@

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////

uniform int texture_space_triangle_size;
void main() {

  @material_input@
  @include "common/gua_global_variable_assignment.glsl"


  gua_color = vec3(0.0, 0.0, 0.0);//pass_color;
  gua_normal = vec3(0.0, 0.0, 1.0);//normalized_normal;

  gua_metalness  = 0.0;
  gua_roughness  = 1.0;
  gua_emissivity = 1.0;
  gua_alpha      = 1.0;
  //gua_flags_passthrough = false;//(gua_emissivity > 0.99999);

  gua_color = texture(color_texture_atlas, pass_uvs).rgb;
  //gua_color = vec3(pass_uvs, 1.0);

  //gua_color = texelFetch(color_texture_atlas, ivec2(gl_FragCoord.xy), 0 ).rgb;
  //gua_color = pass_color;
  //gua_color = vec3(0.0);//texture(color_texture_atlas, pass_uvs).rgb; 
  
  if (gua_rendering_mode != 1) {
   @material_method_calls_frag@
  }

  gua_emissivity = 1.0;
  submit_fragment( gl_FragCoord.z );
  //gua_out_color = vec3(1.0, 0.0, 0.0);
}

