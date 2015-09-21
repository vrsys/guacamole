@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 gua_position_varying;
in vec2 gua_quad_coords;

@input_definition


///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

@uniform_definition


///////////////////////////////////////////////////////////////////////////////
uniform int   using_default_pbr_material;
uniform vec2  win_dims;


layout(binding=0) uniform sampler2D p01_depth_texture;
layout(binding=1) uniform sampler2D p02_color_texture;
layout(binding=2) uniform sampler2D p02_normal_texture;




///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
@output_definition


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() 
{
  vec3  output_color  = vec3(1.0);
  float output_depth  = 1.0f;
  vec3  output_normal = vec3(1.0);

  vec3 coords = vec3(gua_quad_coords, 0.0);


      
        output_color = texture2D( p02_color_texture, coords.xy).rgb;
        output_normal = texture2D( p02_normal_texture, coords.xy).rgb;

        float depthValue = texture2D( p01_depth_texture, coords.xy).r;

	{

		if(depthValue != 0.0f)
                {
		  //out_color = texture2D(p02_color_texture, gl_FragCoord.xy/(win_size.xy));
                  //output_color = vec3(0.0,0.0,1.0);
                  gl_FragDepth = depthValue;
                }
		else
		{

                  
	          float[8] surrounding;
		  surrounding[0] = texture2D(p01_depth_texture, ( (gl_FragCoord.xy + vec2(-1,+1) )/(win_dims.xy)  ) ).r; //upper left pixel
		  surrounding[1] = texture2D(p01_depth_texture, ( (gl_FragCoord.xy + vec2( 0,+1) )/(win_dims.xy)  ) ).r; //upper pixel
		  surrounding[2] = texture2D(p01_depth_texture, ( (gl_FragCoord.xy + vec2(+1,+1) )/(win_dims.xy)  ) ).r; //upper right pixel
		  surrounding[3] = texture2D(p01_depth_texture, ( (gl_FragCoord.xy + vec2(-1, 0) )/(win_dims.xy)  ) ).r; //left pixel
		  surrounding[4] = texture2D(p01_depth_texture, ( (gl_FragCoord.xy + vec2(+1, 0) )/(win_dims.xy)  ) ).r; //right pixel
		  surrounding[5] = texture2D(p01_depth_texture, ( (gl_FragCoord.xy + vec2(-1,-1) )/(win_dims.xy)  ) ).r; //lower left pixel
		  surrounding[6] = texture2D(p01_depth_texture, ( (gl_FragCoord.xy + vec2( 0,-1) )/(win_dims.xy)  ) ).r; //lower pixel
		  surrounding[7] = texture2D(p01_depth_texture, ( (gl_FragCoord.xy + vec2(+1,-1) )/(win_dims.xy)  ) ).r; //lower right pixel
		
		  // surrounding neighbourhood indexing:
		  // 0 1 2
		  // 3   4
		  // 5 6 7

		  //pattern symbols:
		  //b = background pixel
		  //x = random 
		  //o = center pixel
		  
		 //rule 1:
		 //if all of the b-pixel are actually background pixel: pattern matches
		 //rule 2:
		 //if at least 1 pattern matches: don't fill

		  //test against pattern 0  
		  
                  //x b b    x 1 2
 		  //x o b    x   4
		  //x b b    x 6 7
		 
		 bool pattern0 = (surrounding[1] == 1.0) && (surrounding[2] == 1.0) && (surrounding[4] == 1.0) && (surrounding[6] == 1.0) && (surrounding[7] == 1.0) ;
		 
		 //test against pattern 1  
		  
                  //b b b    0 1 2
 		  //b o b    3   4
		  //x x x    x x x
	
		 bool pattern1 = (surrounding[0] == 1.0) && (surrounding[1] == 1.0) && (surrounding[2] == 1.0) && (surrounding[3] == 1.0) && (surrounding[4] == 1.0) ;

		 //test against pattern 2  
		  
                  //b b x    0 1 x
 		  //b o x    3   x
		  //b b x    5 6 x
	
		 bool pattern2 = (surrounding[0] == 1.0) && (surrounding[1] == 1.0) && (surrounding[3] == 1.0) && (surrounding[5] == 1.0) && (surrounding[6] == 1.0) ;

		 //test against pattern 3  
		  
                  //x x x    x x x
 		  //b o b    3   4
		  //b b b    5 6 7
	
		 bool pattern3 = (surrounding[3] == 1.0) && (surrounding[4] == 1.0) && (surrounding[5] == 1.0) && (surrounding[6] == 1.0) && (surrounding[7] == 1.0) ;

		 //test against pattern 4  
		  
                  //b b b    0 1 2
 		  //x o b    x   4
		  //x x b    x x 7
	
		 bool pattern4 = (surrounding[0] == 1.0) && (surrounding[1] == 1.0) && (surrounding[2] == 1.0) && (surrounding[4] == 1.0) && (surrounding[7] == 1.0) ;

		 //test against pattern 5  
		  
                  //b b b    0 1 2
 		  //b o x    3   x
		  //b x x    5 x x
	
		 bool pattern5 = (surrounding[0] == 1.0) && (surrounding[1] == 1.0) && (surrounding[2] == 1.0) && (surrounding[3] == 1.0) && (surrounding[5] == 1.0) ;

		 //test against pattern 6
		  
                  //b x x    0 x x
 		  //b o x    3   x
		  //b b b    5 6 7
	
		 bool pattern6 = (surrounding[0] == 1.0) && (surrounding[3] == 1.0) && (surrounding[5] == 1.0) && (surrounding[6] == 1.0) && (surrounding[7] == 1.0) ;

		 //test against pattern 7
		  
                  //x x b    x x 2
 		  //x o b    x   4
		  //b b b    5 6 7
	
		 bool pattern7 = (surrounding[2] == 1.0) && (surrounding[4] == 1.0) && (surrounding[5] == 1.0) && (surrounding[6] == 1.0) && (surrounding[7] == 1.0) ;



 		  if( pattern0 || pattern1 || pattern2 || pattern3 || pattern4 || pattern5 || pattern6 || pattern7  ) 
		  {
		 	 output_color = vec3(0.f,0.0f,0.0f);
                         output_color = vec3(0.0f,1.0f,0.0f);
                         gl_FragDepth = depthValue;
		  }
		  else
		  {
                  /*
			output_color = vec3(1.0,0.0,0.0);
			




			

			//re-fill the surrounding array with luminosity values of the surrounding area
			vec3 tempCol = vec3(0.0,0.0,0.0);
			tempCol = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(-1,+1) )/(win_dims.xy)  ).rgb; //upper left pixel
			surrounding[0] = 0.2126 * tempCol.r + 0.7152 * tempCol.g + 0.0722 * tempCol.b; 

			tempCol = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(0,+1) )/(win_dims.xy) ).rgb; //upper pixel
			surrounding[1] = 0.2126 * tempCol.r + 0.7152 * tempCol.g + 0.0722 * tempCol.b; 

			tempCol = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(+1,+1) )/(win_dims.xy) ).rgb; //upper right pixel
			surrounding[2] = 0.2126 * tempCol.r + 0.7152 * tempCol.g + 0.0722 * tempCol.b; 

			tempCol = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(-1,0) )/(win_dims.xy) ).rgb; //left pixel
			surrounding[3] = 0.2126 * tempCol.r + 0.7152 * tempCol.g + 0.0722 * tempCol.b; 

			tempCol = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(+1, 0) )/(win_dims.xy) ).rgb; //right pixel
			surrounding[4] = 0.2126 * tempCol.r + 0.7152 * tempCol.g + 0.0722 * tempCol.b; 

			tempCol = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(-1,-1) )/(win_dims.xy) ).rgb; //lower left pixel
			surrounding[5] = 0.2126 * tempCol.r + 0.7152 * tempCol.g + 0.0722 * tempCol.b; 

			tempCol = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(0,-1) )/(win_dims.xy) ).rgb; //lower pixel
			surrounding[6] = 0.2126 * tempCol.r + 0.7152 * tempCol.g + 0.0722 * tempCol.b; 

			tempCol = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(+1,-1) )/(win_dims.xy) ).rgb; //lower right pixel
			surrounding[7] = 0.2126 * tempCol.r + 0.7152 * tempCol.g + 0.0722 * tempCol.b; 

			//find the median element with index 4
			for(int i = 0; i < 8; ++i)
			{

			int sum_smaller_elements = 0;
			int sum_equal_elements = 0;

				for(int k = 0; k < 8; ++k)
				{
					if(i != k)
					{
						if(surrounding[i] < surrounding[k])  //our current element was smaller, so we don't have to do anything
						{//do nothing
						}
						else if(surrounding[i] > surrounding[k])
						{
							sum_smaller_elements += 1;
						}
						else
						{
							sum_equal_elements += 1;
						}
				
					}
				}

				if((sum_smaller_elements +  sum_equal_elements >= 3) )
				{

					//if(renderMode == 0)
					{
						vec3 tempC;
                                                float tempD;
						if( i == 0)
						{
							tempC = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(-1,+1) )/(win_dims.xy) ).rgb;
                                                        tempD = texture2D(p01_depth_texture, (gl_FragCoord.xy + vec2(-1,+1) )/(win_dims.xy) ).r;

						}
						else if(i == 1)
						{
							tempC = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(0,+1) )/(win_dims.xy)).rgb;
                                                        tempD = texture2D(p01_depth_texture, (gl_FragCoord.xy + vec2(0,+1) )/(win_dims.xy) ).r;
						}
						else if(i == 2)
						{
							tempC = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(+1,+1) )/(win_dims.xy)).rgb;
                                                        tempD = texture2D(p01_depth_texture, (gl_FragCoord.xy + vec2(+1,+1) )/(win_dims.xy) ).r;
						}
						else if(i == 3)
						{
							tempC = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(-1, 0) )/(win_dims.xy)).rgb;
                                                        tempD = texture2D(p01_depth_texture, (gl_FragCoord.xy + vec2(-1, 0) )/(win_dims.xy) ).r;
						}
						else if(i == 4)
						{
							tempC = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(+1,0) )/(win_dims.xy) ).rgb;
                                                        tempD = texture2D(p01_depth_texture, (gl_FragCoord.xy + vec2(+1,0) )/(win_dims.xy) ).r;
						}
						else if(i == 5)
						{
							tempC = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(-1,-1) )/(win_dims.xy)).rgb;
                                                        tempD = texture2D(p01_depth_texture, (gl_FragCoord.xy + vec2(-1,-1) )/(win_dims.xy) ).r;
						}
						else if(i == 6)
						{
							tempC = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2( 0,-1) )/(win_dims.xy) ).rgb;
                                                        tempD = texture2D(p01_depth_texture, (gl_FragCoord.xy + vec2( 0,-1) )/(win_dims.xy) ).r;
						}
						else if(i == 7)
						{
							tempC = texture2D(p02_color_texture, (gl_FragCoord.xy + vec2(+1,-1) )/(win_dims.xy)).rgb;
                                                        tempD = texture2D(p01_depth_texture, (gl_FragCoord.xy + vec2(+1,-1) )/(win_dims.xy) ).r;
						}
				
						
						if( (tempC.rgb == vec3(0.0,0.0,0.0) ) && i != 7 )
						{
							continue;
						}
						else
						{
							output_color = tempC;
                                                        output_normal = texture2D(p02_normal_texture, (gl_FragCoord.xy + vec2(+1,-1) )/(win_dims.xy)).rgb;
                                                       // output_color = vec3(1.0,0.0,0.0);

                                                        gl_FragDepth = tempD;
						}
					}

						break;
				} //endif
                               



 

			} //end for

                       



                  */
                  discard;
		  }//end else

                 

		}
                

                
	}

  //gl_FragDepth = 0.5;
  //output_color = vec3(1.0,0.0,0.0);
  //output_color = texture2D( p02_color_texture, coords.xy).rgb;
/////
  {
    @apply_pbr_color
    @apply_pbr_normal
  }


  gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;

////////////////////////////////////////////////////////////////


}
