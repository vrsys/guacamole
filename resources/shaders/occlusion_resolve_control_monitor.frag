@include "common/header.glsl"

// varying input
in vec2 gua_quad_coords;

// uniforms
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_resolve_pass_uniforms.glsl"

// gbuffer input
@include "common/gua_gbuffer_input.glsl"


uniform vec2 downsampling_factors;

layout(binding=0) uniform sampler2D downsampled_depth_buffer;

///////////////////////////////////////////////////////////////////////////////
// output
layout(location=0) out vec3 gua_out_color;


///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
void main() {
  float retrieved_depth = texture(downsampled_depth_buffer, gua_quad_coords).r;

  gua_out_color = vec3(pow(retrieved_depth,4));
  //gua_out_color = vec3(1.0, 0.0, 0.0);
}

