@include "shaders/common/header.glsl"

const float gaussian[32] = float[](
1.000000, 1.000000, 0.988235, 0.968627, 0.956862, 0.917647, 0.894117, 0.870588, 0.915686, 0.788235,
0.749020, 0.690196, 0.654902, 0.619608, 0.552941, 0.513725, 0.490196, 0.458824, 0.392157, 0.356863,
0.341176, 0.278431, 0.254902, 0.227451, 0.188235, 0.164706, 0.152941, 0.125490, 0.109804, 0.098039,
0.074510, 0.062745
);


///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 pass_point_color;  
in vec3 pass_normal;
in float pass_mv_vert_depth;
in float pass_scaled_radius;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

layout (location=0) out vec4 out_accumulated_color;

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
        return gaussian[(int)(round(radius * 32.0))];

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
   //get_gaussianValue(depth_offset, mappedPointCoord, pass_normal);

float depth_to_compare = 0;

   //if(ellipsify)
      depth_to_compare = pass_mv_vert_depth + depth_offset * pass_scaled_radius;
   //else
   // depth_to_compare = pass_mv_vert_depth;


   float weight = get_gaussianValue(depth_offset, mappedPointCoord, pass_normal);

   if( depthValue  - (depth_to_compare)    < 0.00031  + 3.0*(pass_scaled_radius) )
   {
         out_accumulated_color = vec4(pass_point_color * weight, weight);
   }
   else
   {
         discard;
   }


}
