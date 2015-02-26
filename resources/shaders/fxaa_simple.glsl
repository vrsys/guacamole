// simple SSAA adapted from http://www.geeks3d.com/20110405/fxaa-fast-approximate-anti-aliasing-demo-glsl-opengl-test-radeon-geforce/3/

#define SSAA_REDUCE_MIN (1.0/128.0)
#define SSAA_REDUCE_MUL (1.0/8.0)
#define SSAA_SPAN_MAX 8.0

vec4 fxaa_simple (sampler2D color_tex, vec2 fragpos)
{
  vec2 inverse_resolution = vec2(1.0 / float(gua_resolution.x), 1.0 / float(gua_resolution.y));
  vec3 rgbM = texture2D(color_tex, fragpos * inverse_resolution).xyz;

  vec3 rgbNW = texture2D(color_tex, (fragpos + vec2(-1.0, -1.0)) * inverse_resolution).xyz;
  vec3 rgbNE = texture2D(color_tex, (fragpos + vec2(1.0, -1.0)) * inverse_resolution).xyz;
  vec3 rgbSW = texture2D(color_tex, (fragpos + vec2(-1.0, 1.0)) * inverse_resolution).xyz;
  vec3 rgbSE = texture2D(color_tex, (fragpos + vec2(1.0, 1.0)) * inverse_resolution).xyz;

  vec3 simple_luma_ = vec3(0.299, 0.587, 0.114);

  float simple_luma_NW = dot(rgbNW, simple_luma_);
  float simple_luma_NE = dot(rgbNE, simple_luma_);
  float simple_luma_SW = dot(rgbSW, simple_luma_);
  float simple_luma_SE = dot(rgbSE, simple_luma_);
  float simple_luma_M  = dot(rgbM, simple_luma_);

  float simple_luma_Min= min(simple_luma_M, min(min(simple_luma_NW, simple_luma_NE), min(simple_luma_SW, simple_luma_SE)));
  float simple_luma_Max= max(simple_luma_M, max(max(simple_luma_NW, simple_luma_NE), max(simple_luma_SW, simple_luma_SE)));

  vec2 dir;

  dir.x = -((simple_luma_NW+ simple_luma_NE) - (simple_luma_SW+ simple_luma_SE));
  dir.y = ((simple_luma_NW+ simple_luma_SW) - (simple_luma_NE+ simple_luma_SE));

  float dirReduce = max((simple_luma_NW+ simple_luma_NE+ simple_luma_SW+ simple_luma_SE) * (0.25 * SSAA_REDUCE_MUL), SSAA_REDUCE_MIN);

  float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);

  dir = min(vec2(SSAA_SPAN_MAX, SSAA_SPAN_MAX), max(vec2(-SSAA_SPAN_MAX, -SSAA_SPAN_MAX), dir * rcpDirMin)) * inverse_resolution;
  vec3 rgbA = 0.5 * (texture2D(color_tex, fragpos  * inverse_resolution + dir * (1.0 / 3.0 - 0.5)).xyz + texture2D(color_tex, fragpos  * inverse_resolution + dir * (2.0 / 3.0 - 0.5)).xyz);
  vec3 rgbB = rgbA * 0.5 + 0.25 * (texture2D(color_tex, fragpos  * inverse_resolution + dir *  -0.5).xyz + texture2D(color_tex, fragpos  * inverse_resolution + dir * 0.5).xyz);

  float simple_luma_B = dot(rgbB, simple_luma_);

  if ((simple_luma_B < simple_luma_Min) || (simple_luma_B > simple_luma_Max)) {
    return vec4(rgbA, 1.0);
  }
  else {
    return vec4(rgbB, 1.0);
  }
}