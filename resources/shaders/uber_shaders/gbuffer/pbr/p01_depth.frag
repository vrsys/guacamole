@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 gua_point_color;  
in vec3 gua_normal;


///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

layout (location=0) out vec4 out_color;





///////////////////////////////////////////////////////////////////////////////
// splatting methods
///////////////////////////////////////////////////////////////////////////////

float calc_depth_offset(vec2 mappedPointCoord)
{

   vec3 normal = gua_normal;
   if(normal.z < 0)
   {

	//discard;
	normal *= -1;
   }

    float xzRatio = (normal.x/normal.z);
    float yzRatio = (normal.y/normal.z);

//if(clamped_normal_mode)
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


   //get_gaussianValue(depth_offset, mappedPointCoord, VertexIn.nor.xyz);
   get_gaussianValue(depth_offset, mappedPointCoord, gua_normal);


   out_color = vec4(gua_point_color,1.0);


}
