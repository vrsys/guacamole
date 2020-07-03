@include "common/header.glsl"

#if @enable_early_fragment_test@
layout (early_fragment_tests) in;
#endif

@include "common/gua_fragment_shader_input.glsl"
@include "common/gua_camera_uniforms.glsl"

uniform float gua_texel_width;
uniform float gua_texel_height;

vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

@material_uniforms@

@include "common/gua_fragment_shader_output.glsl"
@include "common/gua_global_variable_declaration.glsl"

@include "common/gua_abuffer_collect.glsl"

@material_method_declarations_frag@

#if @enable_virtual_texturing@

@include "common/virtual_texturing_functions.frag"
#endif

void main()
{
  @material_input@
  @include "common/gua_global_variable_assignment.glsl"

  gua_emissivity = 1.0;
  gua_metalness = 0.0;
  gua_color = vec3(1.0, 0.0, 0.0);
  gua_uvs.z = 0.0;
  gua_uvs.w = 0;

  if (gua_rendering_mode == 0) {
  #if @enable_virtual_texturing@
      usampler2D index_texture_mip_map_to_sample = usampler2D(vts[gua_current_vt_idx].vt_address);
      int max_level = vts[gua_current_vt_idx].max_level_and_padding.x;

      vec2 sampled_uv_coords = gua_uvs.xy;
      sampled_uv_coords.y = 1.0 - sampled_uv_coords.y;

      vec4 virtual_texturing_color = traverse_idx_hierarchy(sampled_uv_coords, index_texture_mip_map_to_sample, max_level);
      gua_color = virtual_texturing_color.rgb;
      gua_alpha = virtual_texturing_color.a;
 #endif
 }

  if (gua_rendering_mode != 1) {
      @material_method_calls_frag@
  }

  submit_fragment(gl_FragCoord.z);


}
