@include "common/header.glsl"

//layout(early_fragment_tests) in;

@include "common/gua_camera_uniforms.glsl"

layout (location = 0) out float out_log_depth_texture;

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in VertexData {
  vec2 pass_uv_coords;
  float pass_log_depth;
  float pass_es_linear_depth;
  float pass_es_shift;
} VertexIn;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {
  vec2 uv_coords = VertexIn.pass_uv_coords;

  if( dot(uv_coords, uv_coords) > 1)
    discard;

  //the else branch together with proper shifting avoids depth mismatch between the two depth textures
  if( VertexIn.pass_log_depth >= 0.0 && VertexIn.pass_log_depth  <= 0.9999999 )
    out_log_depth_texture = VertexIn.pass_log_depth; // this goes to gua gbuffers depth texture
  else
  	out_log_depth_texture = 0.0;

  gl_FragDepth = (gl_FragCoord.z * gua_clip_far + VertexIn.pass_es_shift) / gua_clip_far;//( ( -(es_linear_depth_corner + es_shift ) ) / gua_clip_far);; // this is used for depth testing/early z in accum pass

}

