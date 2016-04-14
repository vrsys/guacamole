@include "resources/shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// constants
///////////////////////////////////////////////////////////////////////////////    

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////    
in vec4 v_modelcoord;
in vec2 initial_uv_guess;

in vec3 position_varying;     

flat in int trim_index;
flat in int trim_type;
flat in int trim_approach;
flat in int data_index;

flat in int order_u;
flat in int order_v;

flat in vec4 uvrange;


// generic input
@include "resources/shaders/common/gua_fragment_shader_input.glsl"

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////    
@include "resources/shaders/common/gua_global_variable_declaration.glsl"

@include "resources/shaders/common/gua_fragment_shader_output.glsl"

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////    

@include "resources/shaders/common/gua_camera_uniforms.glsl"

uniform float gua_texel_width;
uniform float gua_texel_height;

// ressource uniforms
uniform samplerBuffer vertexdata;

uniform samplerBuffer trim_partition;
uniform samplerBuffer trim_contourlist;
uniform samplerBuffer trim_curvelist;
uniform samplerBuffer trim_curvedata;
uniform samplerBuffer trim_pointdata;
uniform usamplerBuffer trim_preclassification;

@material_uniforms@

///////////////////////////////////////////////////////////////////////////////
// methods
///////////////////////////////////////////////////////////////////////////////    

#include "resources/glsl/base/compute_depth.frag"
#include "resources/glsl/math/adjoint.glsl.frag"
#include "resources/glsl/math/euclidian_space.glsl.frag"
#include "resources/glsl/math/horner_surface.glsl.frag"
#include "resources/glsl/math/horner_surface_derivatives.glsl.frag"
#include "resources/glsl/math/horner_curve.glsl"
#include "resources/glsl/math/newton_surface.glsl.frag"
#include "resources/glsl/math/raygeneration.glsl.frag" 
#include "resources/glsl/trimming/binary_search.glsl"
#include "resources/glsl/trimming/bisect_curve.glsl"           
#include "resources/glsl/trimming/trimming_contour_double_binary.glsl"
#include "resources/glsl/trimming/trimming_contour_kd.glsl"
#include "resources/glsl/trimming/trimming_double_binary.glsl"
#include "resources/glsl/trimmed_surface/shade_phong_fresnel.glsl.frag"

@include "common/gua_abuffer_collect.glsl"

@material_method_declarations_frag@

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()
{
  int iterations = 16;

  mat4 modelviewmatrix        = gua_view_matrix * gua_model_matrix;
  mat4 modelviewmatrixinverse = inverse(modelviewmatrix);

  /*********************************************************************
  * Ray generation
  *********************************************************************/
  vec3 n1, n2;
  float d1, d2;
  raygen(v_modelcoord, modelviewmatrixinverse, n1, n2, d1, d2);

  /*********************************************************************
  * Surface intersection
  *********************************************************************/
  vec2 uv = initial_uv_guess; 
  
  vec4 p  = vec4(0.0);
  vec4 du = vec4(0.0);
  vec4 dv = vec4(0.0);

  bool surface_hit = newton(uv, 0.0001f, iterations, vertexdata, data_index, order_u, order_v, n1, n2, d1, d2, p, du, dv);

  if ( !surface_hit ) {
    discard;
  } 

  /*********************************************************************
   * Trimming process
   *********************************************************************/
  // transform in NURBS parameter coordinates
  uv[0] = uvrange[0] + uv[0] * (uvrange[1] - uvrange[0]);
  uv[1] = uvrange[2] + uv[1] * (uvrange[3] - uvrange[2]);

  bool trimmed      = trimming_contour_kd (trim_partition,
                                           trim_contourlist,
                                           trim_curvelist,
                                           trim_curvedata,
                                           trim_pointdata,
                                           trim_preclassification,
                                           uv,
                                           int(trim_index), 
                                           trim_type, 
                                           iterations, 
                                           0.0001f, 
                                           16);
  if ( trimmed ) {
    discard;
  }

  /*********************************************************************
   * depth correction
   *********************************************************************/
  @material_input@
  @include "resources/shaders/common/gua_global_variable_assignment.glsl"

  vec4 position_view_space = modelviewmatrix * vec4(p.xyz, 1.0);
  float corrected_depth = compute_depth ( position_view_space, gua_clip_near, gua_clip_far );
  gl_FragDepth = corrected_depth;

  vec3 normal_object_space = normalize(cross(normalize(du.xyz), normalize(dv.xyz)));
  vec3 normal_view_space   = normalize((gua_normal_matrix * vec4(normal_object_space.xyz, 0.0)).xyz);
  
  gua_normal = normal_view_space;
  
  if ( dot(normal_view_space, normalize(-position_view_space.xyz)) < 0.0 )
  {
    gua_normal = -gua_normal;
  }

  @material_method_calls_frag@

  //submit_fragment(gl_FragCoord.z);
  submit_fragment(corrected_depth);
}