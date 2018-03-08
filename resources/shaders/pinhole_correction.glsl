/////////////////////////////////////////////////////////////////////////////
// color hole-filling
/////////////////////////////////////////////////////////////////////////////
bool examine_potential_hole_fill (inout vec4 color, out float depth, in float threshold)
{
  mat4x2 kernel = mat4x2(vec2(1,1), 
                         vec2(0,1), 
                         vec2(1,0), 
                         vec2(-1,1));

  float pixel_depth = texelFetch(sampler2D(gua_gbuffer_depth), ivec2(gl_FragCoord.xy), 0).r;
  vec4  pixel_color = texelFetch(sampler2D(gua_gbuffer_color), ivec2(gl_FragCoord.xy), 0);

  for (int i = 0; i != 4; ++i) 
  {
    float d0 = texelFetch(sampler2D(gua_gbuffer_depth), ivec2(gl_FragCoord.xy + kernel[i]), 0).r;
    float d1 = texelFetch(sampler2D(gua_gbuffer_depth), ivec2(gl_FragCoord.xy - kernel[i]), 0).r;

    vec4 c0 = texelFetch(sampler2D(gua_gbuffer_color), ivec2(gl_FragCoord.xy + ivec2(1,0)), 0);
    vec4 c1 = texelFetch(sampler2D(gua_gbuffer_color), ivec2(gl_FragCoord.xy - ivec2(1,0)), 0);

    if (abs(d0 - d1) < threshold && 
        abs(pixel_depth - d1) > threshold &&
        abs(pixel_depth - d0) > threshold
        )
    {      
      color = (c0 + c1) / 2;
      depth = (d0 + d1) / 2;

      return true;
    }
  }
  return false;
}



/////////////////////////////////////////////////////////////////////////////
// pin-hole-filling
/////////////////////////////////////////////////////////////////////////////
bool examine_color_pinhole (out vec4 fillcolor, float depth_tolerance, float color_tolerance )
{ 
  mat4x2 kernel = mat4x2(vec2(1,1), vec2(1,0), vec2(0,1), vec2(-1,1));
  
  vec3  color = texelFetch(sampler2D(gua_gbuffer_color), ivec2(gl_FragCoord.xy), 0).rgb;
  float depth = texelFetch(sampler2D(gua_gbuffer_depth), ivec2(gl_FragCoord.xy), 0).r;
  
  bool depth_conform = true;
  bool gap_found = false;

  vec4 average_color = vec4(0.0);

  for (int i = 0; i != 4; ++i) 
  {
    float d0 = texelFetch(sampler2D(gua_gbuffer_depth), ivec2(gl_FragCoord.xy + kernel[i]), 0).r;
    float d1 = texelFetch(sampler2D(gua_gbuffer_depth), ivec2(gl_FragCoord.xy - kernel[i]), 0).r;

    vec3 c0 = texelFetch(sampler2D(gua_gbuffer_color), ivec2(gl_FragCoord.xy + kernel[i]), 0).rgb;
    vec3 c1 = texelFetch(sampler2D(gua_gbuffer_color), ivec2(gl_FragCoord.xy - kernel[i]), 0).rgb;

    depth_conform = depth_conform;// && abs((depth - d0) - (d1 - depth)) < depth_tolerance;

    bool same_color = length(c0 - c1) < color_tolerance;
    bool color_gap  = length(c0 - color) > color_tolerance && length(c1 - color) > color_tolerance;

    if (color_gap && same_color) {
      average_color += vec4(c0, 1.0) + vec4(c1, 1.0);
    }

    gap_found = gap_found || (color_gap&& same_color);

    if (!depth_conform) {
      return false;
    }
  }

  if (depth_conform && gap_found) {
    fillcolor = vec4(average_color.rgb/average_color.w, 1.0);
    return true;
  } else {
    return false;
  }
}