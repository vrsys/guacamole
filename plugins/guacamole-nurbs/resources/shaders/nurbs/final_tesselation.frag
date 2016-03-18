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

uniform mat4 gua_view_inverse_matrix;

@include "resources/shaders/common/gua_camera_uniforms.glsl"

#define GPUCAST_HULLVERTEXMAP_SSBO_BINDING 1
#define GPUCAST_ATTRIBUTE_SSBO_BINDING 2

@include "resources/glsl/common/obb_area.glsl"   
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


// normal is assumed to be normalized already
void force_front_facing_normal() 
{
  vec4 C = gua_view_inverse_matrix * vec4(0.0, 0.0, 0.0, 1.0);
  vec3 V = normalize(C.xyz - gua_world_position);
  vec3 N = gua_normal.xyz;

  if (dot(V, N) < 0.0) {
    gua_normal *= -1.0;
  }
}


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
                                           int(trim_index), 1, tmp, 0.0001f, 16);

  // fully discard trimmed fragments
  if ( trimmed ) {
      discard;
  }

  /////////////////////////////////////////////////////
  // 2. app
  /////////////////////////////////////////////////////
  @material_input@
  @include "resources/shaders/common/gua_global_variable_assignment.glsl"

#if 0
  /////////////////////////////////////////////////////
  // 3. correct per-fragment normal and position based on rasterized uv
  /////////////////////////////////////////////////////
  int surface_index;
  int surface_order_u, surface_order_v;

  retrieve_patch_data(int(gIndex), surface_index, surface_order_u, surface_order_v);

  vec2 uv = gua_varying_texcoords.xy;
  vec4 puv;
  vec4 puv_du, puv_dv;
  evaluateSurface(parameter_texture,                                   
                  surface_index,                                  
                  surface_order_u,                                
                  surface_order_v,                                
                  uv, puv, puv_du, puv_dv); 
  
  gua_world_position = (gua_model_matrix * vec4(puv.xyz, 1.0)).xyz;
  vec3 normal_object_space = normalize(cross(normalize(puv_du.xyz), normalize(puv_dv.xyz)));
  gua_normal = normalize((gua_normal_matrix * vec4(normal_object_space, 0.0)).xyz);
#endif

  force_front_facing_normal();

  @material_method_calls_frag@

  gua_color = gua_normal;

  submit_fragment(gl_FragCoord.z);
}