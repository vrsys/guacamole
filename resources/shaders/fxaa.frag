@include "shaders/common/header.glsl"

// varying input
in vec2 gua_quad_coords;

// uniforms
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_resolve_pass_uniforms.glsl"

// gbuffer input
@include "common/gua_gbuffer_input.glsl"

//@include "fxaa_lotthes.glsl"
@include "fxaa_simple.glsl"

// output
layout(location=0) out vec3 gua_out_color;

uniform sampler2D dummy_tex;

void main ()
{
#if 0
  /*============================================================================
  Input variable configuration
  ============================================================================*/
  vec2 fxaaQualityRcpFrame;
  vec4 fxaaConsoleRcpFrameOpt;
  vec4 fxaaConsoleRcpFrameOpt2;
  vec4 fxaaConsole360RcpFrameOpt2;

  //   1.00 - upper limit (softer)
  //   0.75 - default amount of filtering
  //   0.50 - lower limit (sharper, less sub-pixel aliasing removal)
  //   0.25 - almost off
  //   0.00 - completely off
  float fxaaQualitySubpix = gua_ssao_intensity;

  //   0.333 - too little (faster)
  //   0.250 - low quality
  //   0.166 - default
  //   0.125 - high quality 
  //   0.063 - overkill (slower)
  float fxaaQualityEdgeThreshold = 0.063;

  //   0.0833 - upper limit (default, the start of visible unfiltered edges)
  //   0.0625 - high quality (faster)
  //   0.0312 - visible limit (slower)
  float fxaaQualityEdgeThresholdMin = 0.0625;
  float fxaaConsoleEdgeSharpness;
  float fxaaConsoleEdgeThreshold;
  float fxaaConsoleEdgeThresholdMin;
  vec4 fxaaConsole360ConstDir;

  fxaaQualityRcpFrame.x = 1.0 / float(gua_resolution.x);
  fxaaQualityRcpFrame.y = 1.0 / float(gua_resolution.y);

vec4 fxaa_color = FxaaPixelShader(
                                  gua_quad_coords,
                                  vec4(0),
                                  sampler2D(gua_gbuffer_color),
                                  dummy_tex,
                                  dummy_tex,
                                  fxaaQualityRcpFrame,
                                  vec4(1.0),
                                  vec4(1.0),
                                  vec4(1.0),
                                  fxaaQualitySubpix,
                                  fxaaQualityEdgeThreshold,
                                  fxaaQualityEdgeThresholdMin,
                                  1.0,
                                  1.0,
                                  1.0,
                                  vec4(0.0)
                                  );

  gua_out_color = fxaa_color.rgb;

#endif

#if 1
  gua_out_color = fxaa_simple(sampler2D(gua_gbuffer_color), gl_FragCoord.xy).rgb;
#endif
  
}