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
uniform samplerBuffer parameter_texture;      
uniform samplerBuffer attribute_texture;

uniform samplerBuffer trim_partition;
uniform samplerBuffer trim_contourlist;
uniform samplerBuffer trim_curvelist;
uniform samplerBuffer trim_curvedata;
uniform samplerBuffer trim_pointdata;
uniform samplerBuffer trim_preclassification;
uniform bool trim_enabled;

uniform mat4 gua_view_inverse_matrix;

@include "resources/shaders/common/gua_camera_uniforms.glsl"

@include "resources/glsl/trimmed_surface/parametrization_uniforms.glsl"
@include "resources/glsl/common/obb_area.glsl"   

@include "resources/shaders/nurbs/patch_attribute_ssbo.glsl"

@material_uniforms@

///////////////////////////////////////////////////////////////////////////////
// methods
///////////////////////////////////////////////////////////////////////////////    

@material_method_declarations_frag@

@include "resources/glsl/math/horner_curve.glsl"
@include "resources/glsl/math/horner_surface_derivatives.glsl.frag"

@include "resources/glsl/trimming/binary_search.glsl"
@include "resources/glsl/trimming/bisect_curve.glsl"
@include "resources/glsl/trimming/trimming_contour_double_binary.glsl"
@include "resources/glsl/trimming/trimming_contour_kd.glsl"

@include "common/gua_abuffer_collect.glsl"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()
{
  /////////////////////////////////////////////////////
  // 1. perform trimming based on uv value
  /////////////////////////////////////////////////////
  
  // retrieve patch information from ssbo
  vec4 nurbs_domain = retrieve_patch_domain(int(gIndex));
  int trim_index    = retrieve_trim_index(int(gIndex));
  int trim_type     = retrieve_trim_type(int(gIndex));

  // transform bezier coordinates to knot span of according NURBS element
  vec2 domain_size  = vec2(nurbs_domain.z - nurbs_domain.x, nurbs_domain.w - nurbs_domain.y);
  vec2 uv_nurbs     = gua_varying_texcoords.xy * domain_size + nurbs_domain.xy;

  // classify trimming by point-in-curve test
  int tmp;

  bool trimmed      = trimming_contour_kd (trim_partition,
                                           trim_contourlist,
                                           trim_curvelist,
                                           trim_curvedata,
                                           trim_pointdata,
                                           trim_preclassification,
                                           uv_nurbs,
                                           int(trim_index), trim_type, tmp, GPUCAST_TRIMMING_DOMAIN_ERROR_THRESHOLD, GPUCAST_TRIMMING_MAX_BISECTIONS);

  // fully discard trimmed fragments
  if ( trimmed && trim_enabled ) {
      discard;
  }

  /////////////////////////////////////////////////////
  // 2. app
  /////////////////////////////////////////////////////
  @material_input@
  @include "resources/shaders/common/gua_global_variable_assignment.glsl"

  // correct normal to be front-facing
  vec4 nworld  = vec4(gua_normal.xyz, 0.0);
  vec4 nview   = transpose(inverse(gua_view_matrix * gua_model_matrix)) * vec4(gua_normal.xyz, 0.0);
  vec4 cam2pos = gua_view_matrix * vec4(gua_world_position.xyz, 1.0);

  float invert_normal = dot(normalize(nview.xyz), normalize(-cam2pos.xyz)) < 0.0 ? -1.0 : 1.0;
  
  gua_normal = invert_normal * normalize(nworld.xyz);

  @material_method_calls_frag@

  submit_fragment(gl_FragCoord.z);
}