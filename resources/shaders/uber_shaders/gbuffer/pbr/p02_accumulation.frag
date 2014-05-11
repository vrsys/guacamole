@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 pass_point_color;  
in vec3 pass_normal;
in float pass_mv_vert_depth;
in float pass_radius;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

layout (location=0) out vec4 out_color;

///////////////////////////////////////////////////////////////////////////////
//sampler
///////////////////////////////////////////////////////////////////////////////
uniform sampler2D p01_depth_texture;

///////////////////////////////////////////////////////////////////////////////
//Uniforms
///////////////////////////////////////////////////////////////////////////////
uniform float near_plane;
uniform float far_minus_near_plane;
uniform vec2 win_dims;

//uniform float win_dim_x;
//uniform float win_dim_y;

///////////////////////////////////////////////////////////////////////////////
// splatting methods
///////////////////////////////////////////////////////////////////////////////

float calc_depth_offset(vec2 mappedPointCoord)
{

   vec3 normal = pass_normal;
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
                                                                       
   float depthValue = texture2D( p01_depth_texture, gl_FragCoord.xy/win_dims.xy ).r;

   //////////depth value of depth pass texture
   depthValue = (-depthValue * 1.0 * far_minus_near_plane) + near_plane;

   //get_gaussianValue(depth_offset, mappedPointCoord, VertexIn.nor.xyz);
   get_gaussianValue(depth_offset, mappedPointCoord, pass_normal);

float depth_to_compare = 0;

//if(ellipsify)
   depth_to_compare = pass_mv_vert_depth + depth_offset * pass_radius;
//else
// depth_to_compare = pass_mv_vert_depth;

   float weight = 1.0;

   if( depthValue  - (depth_to_compare)    < 0.00031  + 3.0*(pass_radius /** (1/rad_scale_fac)*/ ) )
   {

/*
        float colorCoding = 1.0f;


        if(depthValue != 0.0)
        {
            colorCoding = -depthValue /50.0f;
            colorCoding = 1.0 - colorCoding;
        }

        colorCoding *= 0.3;
        accumulated_colors = vec4(colorCoding * weight, colorCoding * weight, colorCoding * weight, weight);
*/
        //accumulated_colors = vec4(VertexIn.color * weight, weight);
         out_color = vec4(pass_point_color,1.0);
   }
   else
   {
         //out_color = vec4(1.0,0.0,0.0,1.0);
         discard;
   }
//       discard;
   
   //if(win_dim_x == 800.0 && win_dim_y == 600.0)
   /*if(win_dims.x == 800.0 && win_dims.y == 600.0)
     out_color = vec4(0.0,1.0,0.0,1.0);
   else */
    // out_color = vec4(pass_point_color,1.0);


}
