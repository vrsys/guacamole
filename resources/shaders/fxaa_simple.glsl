// simple FXAA adapted from http://www.geeks3d.com/20110405/fxaa-fast-approximate-anti-aliasing-demo-glsl-opengl-test-radeon-geforce/3/

#define FXAA_REDUCE_MIN (1.0/128.0)
#define FXAA_REDUCE_MUL (1.0/8.0)
#define FXAA_SPAN_MAX 8.0

vec4 fxaa_simple (sampler2D color_tex, vec2 fragpos)
{
  vec2 inverse_resolution = vec2(1.0 / float(gua_resolution.x), 1.0 / float(gua_resolution.y));
  vec3 rgbM = texture2D(color_tex, fragpos * inverse_resolution).xyz;

  vec3 rgbNW = texture2D(color_tex, (fragpos + vec2(-1.0, -1.0)) * inverse_resolution).xyz;
  vec3 rgbNE = texture2D(color_tex, (fragpos + vec2(1.0, -1.0)) * inverse_resolution).xyz;
  vec3 rgbSW = texture2D(color_tex, (fragpos + vec2(-1.0, 1.0)) * inverse_resolution).xyz;
  vec3 rgbSE = texture2D(color_tex, (fragpos + vec2(1.0, 1.0)) * inverse_resolution).xyz;

  vec3 luma_ = vec3(0.299, 0.587, 0.114);

  float luma_NW= dot(rgbNW, luma_);
  float luma_NE= dot(rgbNE, luma_);
  float luma_SW= dot(rgbSW, luma_);
  float luma_SE= dot(rgbSE, luma_);
  float luma_M = dot(rgbM, luma_);

  float luma_Min= min(luma_M, min(min(luma_NW, luma_NE), min(luma_SW, luma_SE)));
  float luma_Max= max(luma_M, max(max(luma_NW, luma_NE), max(luma_SW, luma_SE)));

  vec2 dir;

  dir.x = -((luma_NW+ luma_NE) - (luma_SW+ luma_SE));
  dir.y = ((luma_NW+ luma_SW) - (luma_NE+ luma_SE));

  float dirReduce = max((luma_NW+ luma_NE+ luma_SW+ luma_SE) * (0.25 * FXAA_REDUCE_MUL), FXAA_REDUCE_MIN);

  float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);

  dir = min(vec2(FXAA_SPAN_MAX, FXAA_SPAN_MAX), max(vec2(-FXAA_SPAN_MAX, -FXAA_SPAN_MAX), dir * rcpDirMin)) * inverse_resolution;
  vec3 rgbA = 0.5 * (texture2D(color_tex, fragpos  * inverse_resolution + dir * (1.0 / 3.0 - 0.5)).xyz + texture2D(color_tex, fragpos  * inverse_resolution + dir * (2.0 / 3.0 - 0.5)).xyz);
  vec3 rgbB = rgbA * 0.5 + 0.25 * (texture2D(color_tex, fragpos  * inverse_resolution + dir *  -0.5).xyz + texture2D(color_tex, fragpos  * inverse_resolution + dir * 0.5).xyz);

  float luma_B = dot(rgbB, luma_);

  if ((luma_B < luma_Min) || (luma_B > luma_Max)) {
    return vec4(rgbA, 1.0);
  }
  else {
    return vec4(rgbB, 1.0);
  }
}