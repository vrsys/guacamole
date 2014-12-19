@include "resources/shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// constants
///////////////////////////////////////////////////////////////////////////////    

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////    
in vec4 v_modelcoord;
in vec4 frag_texcoord;

in vec3 gua_position_varying;     

flat in int trim_index_db;
flat in int trim_index_cmb;
flat in int data_index;
flat in int order_u;
flat in int order_v;

flat in vec4 uvrange;
flat in int  trimtype;

// generic input
@include "resources/shaders/common/gua_fragment_shader_input.glsl"

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////    
layout (location = 0) out vec4  out_color;
layout (depth_any)    out float gl_FragDepth;

@include "resources/shaders/common/gua_global_variable_declaration.glsl"

@include "resources/shaders/common/gua_fragment_shader_output.glsl"

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////    

@include "resources/shaders/common/gua_camera_uniforms.glsl"

uniform float nearplane;
uniform float farplane;

uniform float gua_texel_width;
uniform float gua_texel_height;

// ressource uniforms
uniform samplerBuffer vertexdata;

uniform samplerBuffer trim_partition;
uniform samplerBuffer trim_contourlist;
uniform samplerBuffer trim_curvelist;
uniform samplerBuffer trim_curvedata;
uniform samplerBuffer trim_pointdata;

@material_uniforms

///////////////////////////////////////////////////////////////////////////////
// methods
///////////////////////////////////////////////////////////////////////////////    

@material_method_declarations

#include "resources/glsl/base/compute_depth.frag"
#include "resources/glsl/math/adjoint.glsl.frag"
#include "resources/glsl/math/euclidian_space.glsl.frag"
#include "resources/glsl/math/horner_surface.glsl.frag"
#include "resources/glsl/math/horner_surface_derivatives.glsl.frag"
#include "resources/glsl/math/horner_curve.glsl.frag"
#include "resources/glsl/math/newton_surface.glsl.frag"
#include "resources/glsl/math/raygeneration.glsl.frag" 
#include "resources/glsl/trimmed_surface/binary_search.glsl.frag"
#include "resources/glsl/trimmed_surface/bisect_curve.glsl.frag"           
#include "resources/glsl/trimmed_surface/trimming_contourmap_binary.glsl.frag"
#include "resources/glsl/trimmed_surface/trimming.glsl.frag"
#include "resources/glsl/trimmed_surface/shade_phong_fresnel.glsl.frag"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()
{
  int iterations = 10;

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
  vec2 uv = vec2(frag_texcoord[0], frag_texcoord[1]);
  
  vec4 p  = vec4(0.0);
  vec4 du = vec4(0.0);
  vec4 dv = vec4(0.0);

  bool surface_hit = newton(uv, 0.001f, iterations, vertexdata, data_index, order_u, order_v, n1, n2, d1, d2, p, du, dv);

  if ( !surface_hit ) {
    discard;
  }

  vec3 normal = normalize(cross(normalize(du.xyz), normalize(dv.xyz)));

  /*********************************************************************
   * Trimming process
   *********************************************************************/
  // transform in NURBS parameter coordinates
  uv[0] = uvrange[0] + uv[0] * (uvrange[1] - uvrange[0]);
  uv[1] = uvrange[2] + uv[1] * (uvrange[3] - uvrange[2]);

  bool trimmed      = trim (trim_partition,
                            trim_contourlist,
                            trim_curvelist,
                            trim_curvedata,
                            trim_pointdata,
                            uv,
                            int(trim_index_cmb), 1, iterations, 0.0001f, 16);
  if ( trimmed ) {
    discard;
  }

  @material_input
  @include "resources/shaders/common/gua_global_variable_assignment.glsl"

  @material_method_calls
  @include "resources/shaders/common/gua_write_gbuffer.glsl"

  /*********************************************************************
   * depth correction
   *********************************************************************/
  vec4 p_world = modelviewmatrix * vec4(p.xyz, 1.0);
  float corrected_depth = compute_depth ( p_world, nearplane, farplane );
  gl_FragDepth = corrected_depth;
}