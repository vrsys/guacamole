@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////




void main() {
  float epsilon = (gua_clip_far - gua_clip_near) / 100000.0;
  gl_FragDepth = ((gl_FragCoord.z * gua_clip_far) / gua_clip_far) + epsilon;
}

