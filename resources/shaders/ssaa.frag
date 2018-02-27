@include "shaders/common/header.glsl"

// varying input
in vec2 gua_quad_coords;

// uniforms
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_resolve_pass_uniforms.glsl"

// gbuffer input
@include "common/gua_gbuffer_input.glsl"

@include "fxaa_lotthes.glsl"
@include "fxaa_simple.glsl"
@include "pinhole_correction.glsl"

// output
layout(location=0) out vec3 gua_out_color;

uniform int gua_ssaa_mode = 0;

//   1.00 - upper limit (softer)
//   0.75 - default amount of filtering
//   0.50 - lower limit (sharper, less sub-pixel aliasing removal)
//   0.25 - almost off
//   0.00 - completely off
uniform float gua_fxaa_quality_subpix = 1.0;

//   0.333 - too little (faster)
//   0.250 - low quality
//   0.166 - default
//   0.125 - high quality 
//   0.063 - overkill (slower)
uniform float gua_fxaa_edge_threshold = 0.125;

//   0.0833 - upper limit (default, the start of visible unfiltered edges)
//   0.0625 - high quality (faster)
//   0.0312 - visible limit (slower)
uniform float gua_fxaa_threshold_min = 0.0625;


void main ()
{
  vec2 inverse_resolution = vec2(1.0/float(gua_resolution.x), 1.0 / float(gua_resolution.y));

  switch (gua_ssaa_mode)
  {
    case 0 :  // SSAA 3.11
      gua_out_color = FxaaPixelShader(gua_quad_coords,
                                      sampler2D(gua_gbuffer_color),
                                      inverse_resolution,
                                      gua_fxaa_quality_subpix,
                                      gua_fxaa_edge_threshold,
                                      gua_fxaa_threshold_min).rgb;
      break; 
    case 1 :  // Simple FXAA
      gua_out_color = fxaa_simple(sampler2D(gua_gbuffer_color), gl_FragCoord.xy).rgb;
      break;
    default :  // No FXAA
      gua_out_color = gua_get_color();
  }

  //////////////////////////
  // pinhole error correction
  //////////////////////////
  if (gua_enable_pinhole_correction) 
  {
    float HOLE_FILLING_THRESHOLD = 0.0001;
    vec4 color = vec4(gua_out_color, 1.0);  
    bool fill_available = false;

    //////////////////////////
    // fill geometric hole
    //////////////////////////
    vec4  fill_color = vec4(0.0);
    float fill_depth = 0.0;

    fill_available = examine_potential_hole_fill(fill_color, fill_depth, HOLE_FILLING_THRESHOLD);

    if (fill_available) {
      color = fill_color;
    }

    //////////////////////////
    // fill color hole
    //////////////////////////
    bool pixel_is_color_hole = examine_color_pinhole(fill_color, HOLE_FILLING_THRESHOLD, 0.4);
    if (pixel_is_color_hole) {
       color = fill_color;
    }
    gua_out_color = color.xyz;
  }


}