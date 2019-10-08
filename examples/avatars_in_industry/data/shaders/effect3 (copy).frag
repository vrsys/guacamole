// https://www.shadertoy.com/view/Mdf3zr
// edge glow, created by jmk in 2013-Feb-16
// adapted for avango by bernstein
//#version 450
#version 440
#extension GL_NV_bindless_texture  : require
#extension GL_ARB_bindless_texture : enable
#extension GL_NV_gpu_shader5       : enable

#extension GL_ARB_shader_storage_buffer_object : enable
#extension GL_ARB_separate_shader_objects : enable

#ifdef GL_NV_shader_atomic_int64
#extension GL_NV_shader_atomic_int64 : enable
#endif
// 4 float layout
layout (std140, binding=0) uniform cameraBlock {
  mat4  gua_view_matrix;
  mat4  gua_projection_matrix;
  mat4  gua_inverse_projection_matrix;
  mat4  gua_inverse_projection_view_matrix;
  vec4  gua_camera_position_4;
  vec4  gua_clipping_planes[64];
  uvec2 gua_resolution;
  uvec2 gua_noise_texture;
  vec4  gua_cyclops_position_4;
  int   gua_clipping_plane_count;
  int   gua_view_id;
  float gua_clip_near;
  float gua_clip_far;
};

vec3 gua_camera_position = gua_camera_position_4.xyz;
vec3 gua_cyclops_position = gua_cyclops_position_4.xyz;

uniform mat4 gua_model_matrix;
uniform mat4 gua_model_view_matrix;
uniform mat4 gua_model_view_inverse_matrix;
uniform mat4 gua_model_view_projection_matrix;
uniform mat4 gua_normal_matrix;
uniform int  gua_rendering_mode; // 0: normal, 1: lowfi shadows, 2: hifi shadows
uniform uvec2 gua_gbuffer_color;
uniform uvec2 gua_gbuffer_pbr;
uniform uvec2 gua_gbuffer_normal;
uniform uvec2 gua_gbuffer_flags;
uniform uvec2 gua_gbuffer_depth;

uniform float gua_texel_width;
uniform float gua_texel_height;

uniform float time;

// quad coords -----------------------------------------------------------------
vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

// color -----------------------------------------------------------------------
vec3 gua_get_color(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_color), frag_pos).rgb;
}

vec3 gua_get_color() {
    return gua_get_color(gua_get_quad_coords());
}

// color -----------------------------------------------------------------------
float gua_get_depth(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).r;
}

float gua_get_depth() {
    return gua_get_depth(gua_get_quad_coords());
}

//layout(location = 0) out vec4 outColor;
layout(location=0) out vec3 gua_out_color;

#define SMOOTH(r) (mix(1.0, 0.0, smoothstep(0.9,1.0, r)))
#define M_PI 3.1415926535897932384626433832795

float movingRing(vec2 uv, vec2 center, float r1, float r2)
{
    vec2 d = uv - center;
    float r = sqrt( dot( d, d ) );
    d = normalize(d);
    float theta = -atan(d.y,d.x);
    theta  = mod(-time+0.5*(1.0+theta/M_PI), 1.0);

    //anti aliasing for the ring's head (thanks to TDM !)
    theta -= max(theta - 1.0 + 1e-2, 0.0) * 1e2;
    return theta*(SMOOTH(r/r2)-SMOOTH(r/r1));
}

float d = sin(1.0)*0.5 + 1.5; // kernel offset

float lookup(vec2 p, float dx, float dy)
{
    //vec2 uv = (p.xy + vec2(dx * d, dy * d)) / iResolution.xy;
    vec2 uv = (p.xy + vec2(dx * d, dy * d)) / vec2(gua_resolution.xy);
    //vec4 c = texture2D(iChannel0, uv.xy);    
    // return as luma
    //return 0.2126*c.r + 0.7152*c.g + 0.0722*c.b;
    
    float c = gua_get_depth(uv.xy) * 100.0;
    return c;

}

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    vec2 p = fragCoord.xy;

    // simple sobel edge detection
    float gx = 0.0;
    gx += -1.0 * lookup(p, -1.0, -1.0);
    gx += -2.0 * lookup(p, -1.0,  0.0);
    gx += -1.0 * lookup(p, -1.0,  1.0);
    gx +=  1.0 * lookup(p,  1.0, -1.0);
    gx +=  2.0 * lookup(p,  1.0,  0.0);
    gx +=  1.0 * lookup(p,  1.0,  1.0);

    float gy = 0.0;
    gy += -1.0 * lookup(p, -1.0, -1.0);
    gy += -2.0 * lookup(p,  0.0, -1.0);
    gy += -1.0 * lookup(p,  1.0, -1.0);
    gy +=  1.0 * lookup(p, -1.0,  1.0);
    gy +=  2.0 * lookup(p,  0.0,  1.0);
    gy +=  1.0 * lookup(p,  1.0,  1.0);

    // hack: use g^2 to conceal noise in the video
    float g = gx*gx + gy*gy;
    float g2 = g * (sin(1.0) / 2.0 + 0.5);

    vec4 col = vec4(gua_get_color(), 1.0);
    col += vec4(0.0, g, g2, 1.0);

    fragColor = col;
}

void main() {
  vec4 color;
  mainImage(color, gl_FragCoord.xy);
  gua_out_color = vec3(color);
}
