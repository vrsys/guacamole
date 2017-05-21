@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_global_variable_declaration.glsl"

layout(binding=0) uniform sampler2D blit_texture;


@include "common/gua_fragment_shader_output.glsl"
//layout(location = 0) out vec3 out_color;


void main() {

  vec4 fetched_texel = texelFetch(blit_texture, ivec2(gl_FragCoord.xy), 0).rgba;
  if(fetched_texel.a == 0.0) {
    discard;
  }

  
  gua_color = vec3(fetched_texel.xyz);


  @include "common/gua_write_gbuffer.glsl"
}