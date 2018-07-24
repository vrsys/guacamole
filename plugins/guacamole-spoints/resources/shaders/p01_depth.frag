@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////


#define ZNEAR gua_clip_near
#define ZFAR gua_clip_far

#define A (ZNEAR + ZFAR)
#define B (ZNEAR - ZFAR)
#define C (2.0 * ZNEAR * ZFAR)
#define D (ndcPos.z * B)
#define ZEYE -(C / (A + D))

float get_lin_eyespace_depth_from_fragment() {

  vec3 ndcPos;
  ndcPos.xy = gl_FragCoord.xy / vec2(gua_resolution.xy);
  ndcPos.z =gl_FragCoord.z;
  ndcPos -= 0.5;
  ndcPos *= 2.0;
  vec4 clipPos;
  clipPos.w = -ZEYE;
  clipPos.xyz = ndcPos * clipPos.w;
  vec4 eyePos = gua_inverse_projection_matrix * clipPos;
  return eyePos.z;
}

//out float gl_FragDepth;
//in float es_half_cube_side_length;
//in float eye_space_depth;
void main() {
  //float epsilon = //(gua_clip_far - gua_clip_near) / 100000.0;
  //gl_FragDepth = (-(get_lin_eyespace_depth_from_fragment() + es_half_cube_side_length)) / gua_clip_far;//((gl_FragCoord.z * gua_clip_far) / gua_clip_far) + epsilon;

  //gl_FragDepth = gl_FragCoord.z;
  //how far should it blend? Voxels overlap slightly and furthermore should not occupy the same space; -> move at most 1 voxel length (for now hardcode to something)
}

