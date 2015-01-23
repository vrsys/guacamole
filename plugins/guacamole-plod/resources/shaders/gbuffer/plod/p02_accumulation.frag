@include "resources/shaders/common/header.glsl"

layout(early_fragment_tests) in;

@include "resources/shaders/common/gua_camera_uniforms.glsl"

const float gaussian[32] = float[](
  1.000000, 1.000000, 0.988235, 0.968627, 0.956862, 0.917647, 0.894117, 0.870588, 0.915686, 0.788235,
  0.749020, 0.690196, 0.654902, 0.619608, 0.552941, 0.513725, 0.490196, 0.458824, 0.392157, 0.356863,
  0.341176, 0.278431, 0.254902, 0.227451, 0.188235, 0.164706, 0.152941, 0.125490, 0.109804, 0.098039,
  0.074510, 0.062745
);

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in VertexData {
  vec3 pass_point_color;
  vec3 pass_normal;
  vec2 pass_uv_coords;
  float pass_es_linear_depth;
  float pass_ms_rad;
} VertexIn;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

layout (location = 0) out vec4 out_accumulated_color;
layout (location = 1) out vec4 out_accumulated_normal;

///////////////////////////////////////////////////////////////////////////////
//sampler
///////////////////////////////////////////////////////////////////////////////
layout(binding=0) uniform sampler2D p01_linear_depth_texture;

///////////////////////////////////////////////////////////////////////////////
// splatting methods
///////////////////////////////////////////////////////////////////////////////
uniform vec2 win_dims;
///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {
  vec2 uv_coords = VertexIn.pass_uv_coords;

  if( dot(uv_coords, uv_coords) > 1) 
    discard;

 
  float neg_pass_1_linear_depth = -texelFetch(p01_linear_depth_texture, ivec2(gl_FragCoord.xy), 0).r;

  float neg_pass_2_linear_depth = VertexIn.pass_es_linear_depth;

  //if(  (neg_pass_1_linear_depth) - (neg_pass_2_linear_depth) >= 0.02*VertexIn.pass_ms_rad)//+ VertexIn.pass_es_linear_depth  > 0.000000001 ) 
  // discard;

//  if(-pass_1_linear_depth > -3.0)
//    discard;

  float normalAdjustmentFactor = 1.0;

  if (VertexIn.pass_normal.z < 0.0) {
    normalAdjustmentFactor = -1.0;
  }



  float weight = gaussian[(int)(round(length(uv_coords) * 31.0))];

  //float weight = 1.0;

   // if( !(render_target_width > 1900.0 && render_target_height > 1000.0 && render_target_width < 2000.0 && render_target_height < 1200) )
      //out_accumulated_color = vec4(1.0, 0.0, 0.0, 1.0);
   // else
  //out_accumulated_color = vec4(VertexIn.pass_point_color * weight, weight);
/*
  if(pass_1_linear_depth > 490.0)
  {
    out_accumulated_color = vec4(1.0 * weight, 0.0, 0.0, weight);
    //out_accumulated_color = vec4(0.0, pass_1_linear_depth, 0.0, 1.0);
    out_accumulated_normal = normalAdjustmentFactor * vec4(weight * VertexIn.pass_normal, weight);
  }
  else
  {
*/
    out_accumulated_color = vec4(weight * VertexIn.pass_point_color, weight);
    //out_accumulated_color = vec4(0.0, 1.0 * weight, 0.0, weight);
    out_accumulated_normal = normalAdjustmentFactor * vec4(weight * VertexIn.pass_normal, weight);
//  }
}

