@include "common/header.glsl"

// varying input
in vec2 gua_quad_coords;

// uniforms

uniform vec2 downsampling_factors;
uniform uvec2 original_resolution;

layout(binding=0) uniform sampler2D gua_in_depth_buffer;

///////////////////////////////////////////////////////////////////////////////
// output
layout(location=0) out float gua_out_downs_depth;


///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
void main() {

  ivec2 frag_pos = ivec2(gl_FragCoord.xy);

  //float read_depth = gua_get_unscaled_depth();

  uvec2 floored_xy = uvec2(floor( max(vec2(0,0), (downsampling_factors*(frag_pos+vec2(0.5) ) - downsampling_factors ) )) );
  uvec2 ceiled_xy  = uvec2(ceil( min(vec2(original_resolution) - vec2(1.0, 1.0), ivec2(downsampling_factors*(frag_pos+vec2(0.5)) + downsampling_factors)) ) );


  float max_depth = 0.0; // initialized with min value

  for (uint y = floored_xy.y; y <= ceiled_xy.y; ++y) {
    for (uint x = floored_xy.x; x <= ceiled_xy.x; ++x) {
      //vec2 normalized_sampling_pos = vec2(x,y));
      float current_depth_sample = texelFetch(gua_in_depth_buffer, ivec2(x,y), 0).r;
      max_depth = max(max_depth, current_depth_sample);
    }
  }


  //gl_FragDepth = gua_get_unscaled_depth();
  //gl_FragDepth = 0.1;

  gua_out_downs_depth = max_depth;
  //gua_out_downs_depth = texelFetch(gua_in_depth_buffer, ivec2( downsampling_factors * frag_pos), 0 ).r;
  //gua_out_downs_depth = texelFetch(gua_in_depth_buffer, ivec2(600, 300), 0 ).r;
 // gua_out_downs_depth = 0.2;
  //texture(gua_depth_buffer, vec2(0.9, 0.5)).r;//max_depth;
  //gl_FragDepth = 0.2;
//  if(read_depth < 0.96) {
    //gua_out_color = vec3(max_depth, 0, 0);
  //} else {
  //  gua_out_color = vec3(0.0, 0.0, 0.0);
  //}
  //*/
  //gua_out_color = vec3(1.0, 0.0, 0.0);
}

