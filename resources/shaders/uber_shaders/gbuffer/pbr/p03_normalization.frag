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

uniform sampler2D p01_depth_texture;
uniform sampler2D p02_color_texture;




///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
@output_definition


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  vec3  output_color  = vec3(1.0);
  float output_depth  = 1.0f;
  vec3  output_normal = vec3(0.0);

  vec3 coords = vec3(gua_quad_coords, 0.0);


  float maxdist = 1000.0;
  float mindist = maxdist;
  float ogldepth = 1.0;
  


      
      vec4 finalcol = texture2D( p02_color_texture, coords.xy);

      output_depth  = ogldepth;
      //output_color  = vec3(finalcol.rg, 1.0);//vec3(0.0,1.0,0.0);//finalcol.rgb;
      output_color = finalcol.rgb / finalcol.a;


  gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;

 // if ( using_default_pbr_material != 0 )
  {
    @apply_pbr_color
    @apply_pbr_normal
  }

  gl_FragDepth = output_depth;
  gl_FragDepth = texture2D( p01_depth_texture, coords.xy).r;
}
