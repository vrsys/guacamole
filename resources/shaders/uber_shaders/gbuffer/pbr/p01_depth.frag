@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 pass_normal;
in float pass_mv_vert_depth;
in float pass_radius;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

// No output other than depth texture


///////////////////////////////////////////////////////////////////////////////
//Uniforms
///////////////////////////////////////////////////////////////////////////////
uniform float near_plane;
uniform float far_minus_near_plane;

///////////////////////////////////////////////////////////////////////////////
// splatting methods
///////////////////////////////////////////////////////////////////////////////

float calc_depth_offset(vec2 mappedPointCoord)
{

   vec3 normal = pass_normal;
   if(normal.z < 0)
   {
	normal *= -1;
   }

    float xzRatio = (normal.x/normal.z);
    float yzRatio = (normal.y/normal.z);

        //if (clampedNormalMode)
        {
	float zBound = 0.35f;//max_deform_ratio;
	float normalZ = normal.z;

	if(normalZ > 0.0)
		normalZ = max(zBound, normalZ);
	else
		normalZ = -max(zBound, -normalZ);

	xzRatio = (normal.x/normalZ);
	yzRatio = (normal.y/normalZ);

        }

	return -(xzRatio)*mappedPointCoord.x   - (yzRatio * mappedPointCoord.y);

}




float get_gaussianValue(float depth_offset, vec2 mappedPointCoord, vec3 newNormalVec)
{

    float radius;
    //if(ellipsify)
    	radius =  mappedPointCoord.x*mappedPointCoord.x + mappedPointCoord.y*mappedPointCoord.y + depth_offset*depth_offset;
    //else
    //	radius =  mappedPointCoord.x*mappedPointCoord.x + mappedPointCoord.y*mappedPointCoord.y ;


    if(radius > 1.0)
	discard;
    else
	return 1.0f;
}


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() 
{

   vec2 mappedPointCoord = gl_PointCoord*2 + vec2(-1.0f, -1.0f);


   float depth_offset = calc_depth_offset(mappedPointCoord) ;


   get_gaussianValue(depth_offset, mappedPointCoord, pass_normal);


 //  if(ellipsify) //map a greater far plane range in case the depth correction overshoots
        gl_FragDepth =  - ( ( (pass_mv_vert_depth + depth_offset * pass_radius ) - near_plane) / (far_minus_near_plane * 1.0f) ) ;
 //  else
 //       gl_FragDepth = - (  ( (pass_mv_vert_depth)  - near_plane) / (far_minus_near_plane * 1.0f));


}
