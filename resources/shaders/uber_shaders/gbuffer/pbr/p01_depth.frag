@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 pass_normal;
in float pass_mv_vert_depth;
in float pass_scaled_radius;
in float pass_view_scaling;

in float pass_log_depth;
///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

// No output other than depth texture
layout (location = 0) out float out_logarithmic_depth;

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////
uniform float near_plane;
uniform float far_minus_near_plane;
uniform float radius_model_scaling;

///////////////////////////////////////////////////////////////////////////////
// splatting methods
///////////////////////////////////////////////////////////////////////////////

float calc_depth_offset(vec2 mappedPointCoord,
                        vec3 adjustedNormal) {

  float xzRatio = adjustedNormal.x / adjustedNormal.z;
  float yzRatio = adjustedNormal.y / adjustedNormal.z;
	float zBound = 0.3; // max_deform_ratio;
	float normalZ = adjustedNormal.z;

	if (normalZ > 0.0)
		normalZ = max(zBound, normalZ);
	else
		normalZ = -max(zBound, -normalZ);

	xzRatio = adjustedNormal.x / normalZ;
	yzRatio = adjustedNormal.y / normalZ;

	return -xzRatio * mappedPointCoord.x - yzRatio * mappedPointCoord.y;
}

float get_gaussianValue(float depth_offset,
                        vec2 mappedPointCoord,
                        vec3 newNormalVec) {

  float radius;

  //if(ellipsify)
  radius =  sqrt(mappedPointCoord.x * mappedPointCoord.x +
                 mappedPointCoord.y * mappedPointCoord.y +
                 depth_offset * depth_offset);
  //else
  //radius = mappedPointCoord.x * mappedPointCoord.x +
  //         mappedPointCoord.y * mappedPointCoord.y ;

  if (radius > 1.0)
    discard;
  else
    return 1.0;
}

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  vec3 adjustedNormal = pass_normal;
  if (pass_normal.z < 0.0) {
    //discard;
    adjustedNormal *= -1.0;
  }

  vec2 mappedPointCoord = gl_PointCoord * 2.0 + vec2(-1.0);

  float depth_offset = calc_depth_offset(mappedPointCoord, adjustedNormal);
  get_gaussianValue(depth_offset, mappedPointCoord, adjustedNormal);
  
/*out_linear_depth.r = -((pass_mv_vert_depth + depth_offset *
                        pass_scaled_radius * pass_view_scaling) - near_plane) /
                      (far_minus_near_plane * 1.0);
*/
  out_logarithmic_depth = pass_log_depth;
  //out_logarithmic_depth = 0.1;

  gl_FragDepth = -((pass_mv_vert_depth + depth_offset *
                        pass_scaled_radius * pass_view_scaling) - near_plane) /
                      (far_minus_near_plane * 1.0);
}

