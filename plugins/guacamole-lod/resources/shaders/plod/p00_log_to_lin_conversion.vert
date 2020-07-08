@include "resources/shaders/common/header.glsl"


///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
layout(location = 0) in vec3 gua_in_position;

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {
  gl_Position = vec4(gua_in_position, 1.0);
}