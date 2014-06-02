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
in float pass_screen_space_splat_size;
in float pass_view_scaling;

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

uniform float radius_model_scaling;

//uniform float win_dim_x;
//uniform float win_dim_y;

///////////////////////////////////////////////////////////////////////////////
// splatting methods
///////////////////////////////////////////////////////////////////////////////

float calc_depth_offset(vec2 mappedPointCoord, vec3 adjustedNormal)
{

    float xzRatio = (adjustedNormal.x/adjustedNormal.z);
    float yzRatio = (adjustedNormal.y/adjustedNormal.z);

if(true)
{
	float zBound = 0.3f;//max_deform_ratio;
	float normalZ = adjustedNormal.z;

	if(normalZ > 0.0)
		normalZ = max(zBound, normalZ);
	else
		normalZ = -max(zBound, -normalZ);

	xzRatio = (adjustedNormal.x/normalZ);
	yzRatio = (adjustedNormal.y/normalZ);

}

	return -(xzRatio)*mappedPointCoord.x   - (yzRatio * mappedPointCoord.y);

}




float get_gaussianValue(float depth_offset, vec2 mappedPointCoord, vec3 newNormalVec)
{

    float radius;

    	radius = sqrt(mappedPointCoord.x*mappedPointCoord.x + mappedPointCoord.y*mappedPointCoord.y + depth_offset*depth_offset);



    if(radius > 1.0)
        discard;
    else
        return gaussian[(int)(round(radius * 31.0))];

}


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() 
{
 
   vec3 adjustedNormal = pass_normal;
       // adjustedNormal = vec3(0.0, 1.0, 0.0);

   if(pass_normal.z < 0)
   {
	//discard;
	adjustedNormal = adjustedNormal * -1;
   }
   else
   {
        adjustedNormal = adjustedNormal;
   }

   vec2 mappedPointCoord = gl_PointCoord*2 + vec2(-1.0f, -1.0f);


   float depth_offset = calc_depth_offset(mappedPointCoord, adjustedNormal) ;
                                                                       
   float depthValue = texture2D( p01_depth_texture, gl_FragCoord.xy/win_dims.xy ).r;

   //////////depth value of depth pass texture
   depthValue = (-depthValue * 1.0 * far_minus_near_plane) + near_plane;


float depth_to_compare = 0;

   depth_to_compare = pass_mv_vert_depth + depth_offset * pass_scaled_radius * pass_view_scaling;


   float weight = 0;

   weight = get_gaussianValue(depth_offset, mappedPointCoord, adjustedNormal) ;

 
   if( depthValue  - (depth_to_compare)    <=  3 * pass_scaled_radius )
   {

      out_accumulated_color = vec4(pass_point_color * weight, weight);
   }
   else
   {
         discard;
   }



}
