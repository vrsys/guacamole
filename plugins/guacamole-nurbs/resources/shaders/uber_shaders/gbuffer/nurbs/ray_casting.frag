 #version 420 core
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : enable

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
@input_definition

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////    
layout (location = 0) out vec4  out_color;
layout (depth_any)    out float gl_FragDepth;

@output_definition

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////    

// uniforms
layout (std140, binding=0) uniform cameraBlock
{
  mat4 gua_view_matrix;
  mat4 gua_projection_matrix;
  mat4 gua_inverse_projection_matrix;
  mat4 gua_inverse_projection_view_matrix;
  vec3 gua_camera_position;
};

uniform float nearplane;
uniform float farplane;

// built-in uniforms
uniform mat4 gua_model_matrix;
uniform mat4 gua_normal_matrix;

uniform float gua_texel_width;
uniform float gua_texel_height;

// ressource uniforms
uniform samplerBuffer vertexdata;

uniform samplerBuffer trim_partition;
uniform samplerBuffer trim_contourlist;
uniform samplerBuffer trim_curvelist;
uniform samplerBuffer trim_curvedata;
uniform samplerBuffer trim_pointdata;

// generic uniforms
@uniform_definition

///////////////////////////////////////////////////////////////////////////////
// methods
///////////////////////////////////////////////////////////////////////////////    
#include "resources/shaders/uber_shaders/common/get_sampler_casts.glsl"

vec2 gua_get_quad_coords() {
    return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

uint gua_get_material_id() {
    return gua_uint_gbuffer_varying_0.x;
}

vec3 gua_get_position() {
    return gua_position_varying;
}

@material_methods

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

  @material_switch

  /*********************************************************************
   * depth correction
   *********************************************************************/
  vec4 p_world = modelviewmatrix * vec4(p.xyz, 1.0);
  float corrected_depth = compute_depth ( p_world, nearplane, farplane );
  gl_FragDepth = corrected_depth;

  /*********************************************************************
   * set built-in output
   *********************************************************************/
  gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;
}