@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////

layout(binding=0) uniform sampler2D blit_texture;


layout(location = 0) out vec3 out_color;


void main() {

  vec4 fetched_texel = texelFetch(blit_texture, ivec2(gl_FragCoord.xy), 0).rgba;
  if(fetched_texel.a == 0.0) {
    discard;
  }

  out_color = vec3(fetched_texel.xyz) *fetched_texel.a;

  //out_color = fetched_texel.rgba;
  //out_color = vec3(gl_FragCoord.xy/1000, 0.0);
}