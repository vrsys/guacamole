@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_global_variable_declaration.glsl"
///////////////////////////////////////////////
////////////////////////////////

@include "common/gua_fragment_shader_output.glsl"

layout(binding  = 0) uniform sampler2D in_color_texture;
layout(binding  = 1) uniform sampler2D in_normal_texture;
layout(binding  = 2) uniform sampler2D in_pbr_texture;
layout(binding = 3, R32UI) readonly coherent uniform restrict uimage2D depth_image;
//layout(location = 0) out vec4 out_color;
        
uniform vec2 win_size;

//for texture access
in vec2 pos;
#define MAX_INT_32 2147483647

float floatify_uint_depth(in uint uint_depth) {
  return float( ( float((MAX_INT_32 - uint_depth)) / float(MAX_INT_32) ) * (gua_clip_far-gua_clip_near) + gua_clip_near);
}

float depthSample(float linearDepth)
{
    float nonLinearDepth = (gua_clip_far + gua_clip_near - 2.0 * gua_clip_near * gua_clip_far / linearDepth) / (gua_clip_far - gua_clip_near);
    nonLinearDepth = (nonLinearDepth + 1.0) / 2.0;
    return nonLinearDepth;
}

void fetch_neighborhood_depth( inout uint[8] in_out_neighborhood ) {
  in_out_neighborhood[0] = imageLoad(depth_image, ivec2(gl_FragCoord.xy + ivec2(-1,+1) ) ).r;
  in_out_neighborhood[1] = imageLoad(depth_image, ivec2(gl_FragCoord.xy + ivec2( 0,+1) ) ).r;
  in_out_neighborhood[2] = imageLoad(depth_image, ivec2(gl_FragCoord.xy + ivec2(+1,+1) ) ).r;
  in_out_neighborhood[3] = imageLoad(depth_image, ivec2(gl_FragCoord.xy + ivec2(-1, 0) ) ).r;
  in_out_neighborhood[4] = imageLoad(depth_image, ivec2(gl_FragCoord.xy + ivec2(+1, 0) ) ).r;
  in_out_neighborhood[5] = imageLoad(depth_image, ivec2(gl_FragCoord.xy + ivec2(-1,-1) ) ).r;
  in_out_neighborhood[6] = imageLoad(depth_image, ivec2(gl_FragCoord.xy + ivec2( 0,-1) ) ).r;
  in_out_neighborhood[7] = imageLoad(depth_image, ivec2(gl_FragCoord.xy + ivec2(+1,-1) ) ).r;
}

void main() {

  uint depthValue = imageLoad(depth_image, ivec2(gl_FragCoord.xy ) ).r;
  float convertedDepth = 1.0; 

  vec3 out_color = vec3(0.0, 0.0, 0.0);

  {

    if(depthValue != 0) {
      out_color = texelFetch(in_color_texture,
                              ivec2(gl_FragCoord.xy), 0).rgb;

      convertedDepth = depthSample( floatify_uint_depth( depthValue ) );
    }
    else
    {
        
        uint[8] neighborhood_depth;

        fetch_neighborhood_depth(neighborhood_depth);

    
      // neighborhood_depth neighbourhood indexing:
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
     
     bool pattern0 = (neighborhood_depth[1] == 0) && (neighborhood_depth[2] == 0) && (neighborhood_depth[4] == 0) && (neighborhood_depth[6] == 0) && (neighborhood_depth[7] == 0) ;
     
     //test against pattern 1  
      
                  //b b b    0 1 2
      //b o b    3   4
      //x x x    x x x
  
     bool pattern1 = (neighborhood_depth[0] == 0) && (neighborhood_depth[1] == 0) && (neighborhood_depth[2] == 0) && (neighborhood_depth[3] == 0) && (neighborhood_depth[4] == 0) ;

     //test against pattern 2  
      
                  //b b x    0 1 x
      //b o x    3   x
      //b b x    5 6 x
  
     bool pattern2 = (neighborhood_depth[0] == 0) && (neighborhood_depth[1] == 0) && (neighborhood_depth[3] == 0) && (neighborhood_depth[5] == 0) && (neighborhood_depth[6] == 0) ;

     //test against pattern 3  
      
                  //x x x    x x x
      //b o b    3   4
      //b b b    5 6 7
  
     bool pattern3 = (neighborhood_depth[3] == 0) && (neighborhood_depth[4] == 0) && (neighborhood_depth[5] == 0) && (neighborhood_depth[6] == 0) && (neighborhood_depth[7] == 0) ;

     //test against pattern 4  
      
                  //b b b    0 1 2
      //x o b    x   4
      //x x b    x x 7
  
     bool pattern4 = (neighborhood_depth[0] == 0) && (neighborhood_depth[1] == 0) && (neighborhood_depth[2] == 0) && (neighborhood_depth[4] == 0) && (neighborhood_depth[7] == 0) ;

     //test against pattern 5  
      
                  //b b b    0 1 2
      //b o x    3   x
      //b x x    5 x x
  
     bool pattern5 = (neighborhood_depth[0] == 0) && (neighborhood_depth[1] == 0) && (neighborhood_depth[2] == 0) && (neighborhood_depth[3] == 0) && (neighborhood_depth[5] == 0) ;

     //test against pattern 6
      
                  //b x x    0 x x
      //b o x    3   x
      //b b b    5 6 7
  
     bool pattern6 = (neighborhood_depth[0] == 0) && (neighborhood_depth[3] == 0) && (neighborhood_depth[5] == 0) && (neighborhood_depth[6] == 0) && (neighborhood_depth[7] == 0) ;

     //test against pattern 7
      
                  //x x b    x x 2
      //x o b    x   4
      //b b b    5 6 7
  
     bool pattern7 = (neighborhood_depth[2] == 0) && (neighborhood_depth[4] == 0) && (neighborhood_depth[5] == 0) && (neighborhood_depth[6] == 0) && (neighborhood_depth[7] == 0) ;


    //red means: is background and should be filled
    //yellow means: is background and should not be filled

      if( pattern0 || pattern1 || pattern2 || pattern3 || pattern4 || pattern5 || pattern6 || pattern7  ) 
      {
      }
      else
      {
        out_color = vec3(1.0,0.0,0.0);

        uint num_accumulated_colors = 0;
        vec3 accumulated_color = vec3(0.0, 0.0, 0.0);
        float unconverted_accum_depth = 0.0;

        if(neighborhood_depth[0] != 0) {
          accumulated_color += texelFetch(in_color_texture,
                                          ivec2(gl_FragCoord.xy + vec2(-1, +1) ), 0).rgb;

          unconverted_accum_depth += float(neighborhood_depth[0]) / 8.0;

          ++num_accumulated_colors;
        }
        if(neighborhood_depth[1] != 0) {
          accumulated_color += texelFetch(in_color_texture,
                                          ivec2(gl_FragCoord.xy + vec2( 0, +1) ), 0).rgb;

          unconverted_accum_depth += float(neighborhood_depth[1]) / 8.0;

          ++num_accumulated_colors;
        }

        if(neighborhood_depth[2] != 0) {
          accumulated_color += texelFetch(in_color_texture,
                                          ivec2(gl_FragCoord.xy + vec2( +1, +1) ), 0).rgb;

          unconverted_accum_depth += float(neighborhood_depth[2]) / 8.0;

          ++num_accumulated_colors;
        }

        if(neighborhood_depth[3] != 0) {
          accumulated_color += texelFetch(in_color_texture,
                                          ivec2(gl_FragCoord.xy + vec2( -1, 0) ), 0).rgb;

          unconverted_accum_depth += float(neighborhood_depth[3]) / 8.0;

          ++num_accumulated_colors;
        }

        if(neighborhood_depth[4] != 0) {
          accumulated_color += texelFetch(in_color_texture,
                                          ivec2(gl_FragCoord.xy + vec2(+1, 0) ), 0).rgb;

          unconverted_accum_depth += float(neighborhood_depth[4]) / 8.0;

          ++num_accumulated_colors;
        }

        if(neighborhood_depth[5] != 0) {
          accumulated_color += texelFetch(in_color_texture,
                                          ivec2(gl_FragCoord.xy + vec2( -1, -1) ), 0).rgb;

          unconverted_accum_depth += float(neighborhood_depth[5]) / 8.0;

          ++num_accumulated_colors;
        }

        if(neighborhood_depth[6] != 0) {
          accumulated_color += texelFetch(in_color_texture,
                                          ivec2(gl_FragCoord.xy + vec2( 0,-1) ), 0).rgb;

          unconverted_accum_depth += float(neighborhood_depth[6]) / 8.0;

          ++num_accumulated_colors;
        }

        if(neighborhood_depth[7] != 0) {
          accumulated_color += texelFetch(in_color_texture,
                                          ivec2(gl_FragCoord.xy + vec2( +1, -1) ), 0).rgb;

          unconverted_accum_depth += float(neighborhood_depth[7]) / 8.0;

          ++num_accumulated_colors;
        }

        if(num_accumulated_colors == 0)
          out_color = vec3(1.0, 1.0, 0.0);
        else
          //out_color = vec4(1.0, 1.0, 1.0, 1.0);
          out_color = vec3(accumulated_color / num_accumulated_colors);

          convertedDepth = depthSample( floatify_uint_depth( uint(num_accumulated_colors * unconverted_accum_depth) ) );
      }


      
      
    }





  }

  gua_out_color  = out_color;

  gua_out_pbr = texelFetch(in_pbr_texture,
                           ivec2(gl_FragCoord.xy), 0).xyz;
  
  gua_out_normal = texelFetch(in_normal_texture,
                           ivec2(gl_FragCoord.xy), 0).xyz;

  gua_out_pbr = vec3(1.0, 1.0, 0.0);

  gl_FragDepth = convertedDepth;



 }

///////////////////////////////////////////////////////////////////////////////
// output


