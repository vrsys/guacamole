@include "resources/shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// constants
///////////////////////////////////////////////////////////////////////////////    
#define TRIM_ERROR_TOLERANCE 0.00001

precision highp float;

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////    
flat in uint gIndex;

@include "resources/shaders/common/gua_fragment_shader_input.glsl"

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////    

@include "resources/shaders/common/gua_global_variable_declaration.glsl"

@include "resources/shaders/common/gua_fragment_shader_output.glsl"

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////    
uniform samplerBuffer attribute_texture;

uniform samplerBuffer trim_partition;
uniform samplerBuffer trim_contourlist;
uniform samplerBuffer trim_curvelist;
uniform samplerBuffer trim_curvedata;
uniform samplerBuffer trim_pointdata;

@include "resources/shaders/common/gua_camera_uniforms.glsl"

@material_uniforms

///////////////////////////////////////////////////////////////////////////////
// methods
///////////////////////////////////////////////////////////////////////////////    

@material_method_declarations

@include "resources/glsl/math/horner_curve.glsl.frag"
@include "resources/glsl/trimmed_surface/binary_search.glsl.frag"
@include "resources/glsl/trimmed_surface/bisect_curve.glsl.frag"
@include "resources/glsl/trimmed_surface/trimming_contourmap_binary.glsl.frag"


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()
{
  vec4 data = texelFetch(attribute_texture, int(gIndex) * 5);
  uint trim_index = floatBitsToUint(data.w);

  vec4 nurbs_domain = texelFetch(attribute_texture, int(gIndex) * 5 + 1);

  vec2 domain_size  = vec2(nurbs_domain.z - nurbs_domain.x, nurbs_domain.w - nurbs_domain.y);

  vec2 uv_nurbs     = gua_varying_texcoords.xy * domain_size + nurbs_domain.xy;

  int tmp = 0;
  bool trimmed      = trim (trim_partition,
                            trim_contourlist,
                            trim_curvelist,
                            trim_curvedata,
                            trim_pointdata,
                            uv_nurbs,
                            int(trim_index), 1, tmp, 0.0001f, 16);
  if ( trimmed ) {
      discard;
  }

  @material_input
  @include "resources/shaders/common/gua_global_variable_assignment.glsl"

  @material_method_calls
  @include "resources/shaders/common/gua_write_gbuffer.glsl"
}