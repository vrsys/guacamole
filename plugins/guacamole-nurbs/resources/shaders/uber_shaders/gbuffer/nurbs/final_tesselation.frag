 #version 420 core
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : enable

///////////////////////////////////////////////////////////////////////////////
// constants
///////////////////////////////////////////////////////////////////////////////    
#define TRIM_ERROR_TOLERANCE 0.00001

precision highp float;

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////    
flat in uint gIndex;
in vec2      gTessCoord;
in vec3      gua_position_varying;

// generic input
@input_definition

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////    
@output_definition

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////    
uniform samplerBuffer attribute_texture;

uniform samplerBuffer trim_partition;
uniform samplerBuffer trim_contourlist;
uniform samplerBuffer trim_curvelist;
uniform samplerBuffer trim_curvedata;
uniform samplerBuffer trim_pointdata;

// uniforms
layout (std140, binding=0) uniform cameraBlock
{
  mat4 gua_view_matrix;
  mat4 gua_projection_matrix;
  mat4 gua_inverse_projection_matrix;
  mat4 gua_inverse_projection_view_matrix;
  vec3 gua_camera_position;
};

uniform mat4 gua_model_matrix;
uniform mat4 gua_normal_matrix;

uniform float gua_texel_width;
uniform float gua_texel_height;

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

#include "resources/glsl/math/horner_curve.glsl.frag"
#include "resources/glsl/trimmed_surface/binary_search.glsl.frag"
#include "resources/glsl/trimmed_surface/bisect_curve.glsl.frag"
#include "resources/glsl/trimmed_surface/trimming_contourmap_binary.glsl.frag"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()
{
  vec4 data = texelFetch(attribute_texture, int(gIndex) * 5);
  uint trim_index = floatBitsToUint(data.w);

  vec4 nurbs_domain = texelFetch(attribute_texture, int(gIndex) * 5 + 1);

  vec2 domain_size  = vec2(nurbs_domain.z - nurbs_domain.x, nurbs_domain.w - nurbs_domain.y);

  vec2 uv_nurbs     = gTessCoord.xy * domain_size + nurbs_domain.xy;

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

  @material_switch

  gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;
}