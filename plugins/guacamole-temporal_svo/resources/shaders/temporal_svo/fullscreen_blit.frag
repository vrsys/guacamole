@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_global_variable_declaration.glsl"

layout(binding=0) uniform sampler2D blit_texture;
layout(binding=1) uniform sampler2D original_gbuffer_color;


@include "common/gua_fragment_shader_output.glsl"
//layout(location = 0) out vec3 out_color;


void main() {

  vec4 fetched_texel = texelFetch(blit_texture, ivec2(gl_FragCoord.xy), 0).rgba;
  if(fetched_texel.a == 0.0) {
    discard;
  }


  vec3 src    =  texelFetch(original_gbuffer_color, ivec2(gl_FragCoord.xy), 0).rgb;
  vec4 color  =  texelFetch(blit_texture, ivec2(gl_FragCoord.xy), 0).rgba;

  //vec4 color = vec4(texelFetch(front_to_back_blending_texture, ivec2(gl_FragCoord.xy), 0) );

  float omda_sa = (1.0 - color.a) * 1.0; //the 1.0 is the src.a, which we can only assume to be 1

  color.rgb += omda_sa * src.rgb;
  color.a   += omda_sa;

  gua_color = color.rgb;
  

  //gua_color = vec3(fetched_texel.xyz) * 0.5 + 0.5 * texelFetch(original_gbuffer_color, ivec2(gl_FragCoord.xy), 0).rgb;
  @include "common/gua_write_gbuffer.glsl"
}