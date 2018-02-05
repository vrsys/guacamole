@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////

layout(binding=0) uniform sampler2D in_color_texture_current;
layout(binding=1) uniform sampler2D in_color_texture_previous;
layout(binding=2) uniform sampler2D in_depth_texture;

layout(location = 0) out vec4 out_color;
//layout(location = 0) out vec3 out_color;


void main() {
/*
  vec4 screen_space_cube_fragment_pos = gua_model_view_projection_matrix * vec4(FragmentIn.pos_ms, 1);
  screen_space_cube_fragment_pos /= screen_space_cube_fragment_pos.w;

  float current_pass_depth = screen_space_cube_fragment_pos.z * 0.5 + 0.5;
  

  vec4 previous_color = texelFetch(in_color_texture, ivec2(gl_FragCoord.xy), 0);
  float previous_depth = texelFetch(in_depth_texture, ivec2(gl_FragCoord.xy), 0).r;

  vec4 src = vec4(0);
  vec4 dst = vec4(0);

  if(previous_depth < current_pass_depth) {
    src = previous_color;
    dst = current_pass_color;
  } else {
    src = current_pass_color;
    dst = previous_color;

    //overwrite frontmost depth value
    gl_FragDepth = current_pass_depth;
  }

  float omda_sa = (1.0 - dst.a) * 1.0; //the 1.0 is the src.a, which we can only assume to be 1

  dst.rgb += omda_sa * src.rgb;
  dst.a   += omda_sa;

  out_color = dst.rgba;

  out_color = current_pass_color + previous_color;

*/
  out_color = texelFetch(in_color_texture_current, ivec2(gl_FragCoord.xy), 0) + texelFetch(in_color_texture_previous, ivec2(gl_FragCoord.xy), 0);

}