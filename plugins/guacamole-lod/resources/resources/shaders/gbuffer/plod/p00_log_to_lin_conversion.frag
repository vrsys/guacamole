@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"


///////////////////////////////////////////////////////////////////////////////
//sampler
///////////////////////////////////////////////////////////////////////////////
layout(binding=0) uniform sampler2D gua_log_depth_buffer;

/*
float linearize_log_depth(float in_log_depth) {

  float lin_z = (2.0 * gua_clip_near * gua_clip_far) / (gua_clip_far + gua_clip_near - in_log_depth * (gua_clip_far - gua_clip_near));

  return lin_z;
}
*/

#define ZNEAR gua_clip_near
#define ZFAR gua_clip_far

#define A (ZNEAR + ZFAR)
#define B (ZNEAR - ZFAR)
#define C (2.0 * ZNEAR * ZFAR)
#define D (ndcPos.z * B)
#define ZEYE -(C / (A + D))

/*
float get_eyespace_depth_from_sampler() {
  vec2 xy = vec2(gl_FragCoord.xy/(vec2(1920.0, 1080.0))); //in [0,1] range
  vec4 v_screen = vec4(xy, texture2D(gua_log_depth_buffer,xy).r, 1.0 );
  v_screen.xyz = 2.0*(v_screen.xyz-vec3(0.5) );
  vec4 v_view = gua_inverse_projection_matrix * v_screen;
  float view_depth = v_view.z / v_view.w; //transfer from homogeneous coordinates
  return view_depth;
}
*/

  uniform float win_width;
  uniform float win_height;

float get_eyespace_depth_from_sampler() {

	vec3 ndcPos;
	//ndcPos.xy = gl_FragCoord.xy / (vec2(1920.0, 1080.0));
  ndcPos.xy = gl_FragCoord.xy / vec2(win_width, win_height);
	ndcPos.z = texture2D (gua_log_depth_buffer, ndcPos.xy).r; // or gl_FragCoord.z
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

