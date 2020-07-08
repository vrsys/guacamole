@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"
// gbuffer input

///////////////////////////////////////////////////////////////////////////////
//sampler
///////////////////////////////////////////////////////////////////////////////
layout(binding=0) uniform sampler2D gua_log_depth_buffer;

#define ZNEAR gua_clip_near
#define ZFAR gua_clip_far

#define A (ZNEAR + ZFAR)
#define B (ZNEAR - ZFAR)
#define C (2.0 * ZNEAR * ZFAR)
#define D (ndcPos.z * B)
#define ZEYE -(C / (A + D))

  uniform float win_width;
  uniform float win_height;

float get_eyespace_depth_from_sampler() {

	vec3 ndcPos;
  	ndcPos.xy = gl_FragCoord.xy / vec2(win_width, win_height);
	//ndcPos.xy = gua_get_quad_coords();
	ndcPos.z = texture2D (gua_log_depth_buffer, ndcPos.xy).r; // or gl_FragCoord.z
	//ndcPos.z = gua_get_depth();

	ndcPos -= 0.5;
	ndcPos *= 2.0;
	vec4 clipPos;
	clipPos.w = -ZEYE;
	clipPos.xyz = ndcPos * clipPos.w;
	vec4 eyePos = gua_inverse_projection_matrix * clipPos;
	return eyePos.z;
}
///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {
  
  float compressed_depth = -(get_eyespace_depth_from_sampler() / gua_clip_far);

  gl_FragDepth = (compressed_depth < 1.0) ? compressed_depth : 1.0;
}