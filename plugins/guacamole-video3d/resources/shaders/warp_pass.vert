@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
layout(location=0) in vec3 gua_in_position;
#if 0
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;
#endif

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// video3d uniforms
///////////////////////////////////////////////////////////////////////////////

uniform sampler2DArray depth_video3d_texture;
uniform sampler3D cv_xyz;
uniform sampler3D cv_uv;
uniform float cv_min_d;
uniform float cv_max_d;
uniform vec2 tex_size_inv;
uniform int layer;

///////////////////////////////////////////////////////////////////////////////
// outputs
///////////////////////////////////////////////////////////////////////////////
out VertexData {
    vec2 texture_coord;
    vec3 pos_es;
    vec3 pos_cs;
    float depth;
    float lateral_quality;
} VertexOut;

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  vec3 coords = vec3(gua_in_position.xy,layer);
  vec2 bf_result          = texture(depth_video3d_texture, coords).rg;
  float depth             = bf_result.x;

  // lookup from calibvolume
  float d_idx = (depth - cv_min_d)/(cv_max_d - cv_min_d);
  VertexOut.pos_cs        = texture(cv_xyz, vec3(gua_in_position.xy, d_idx)).rgb;
  VertexOut.pos_es        = (gua_view_matrix * gua_model_matrix * vec4(VertexOut.pos_cs, 1.0)).xyz;
  VertexOut.texture_coord = texture(cv_uv,  vec3(gua_in_position.xy, d_idx)).rg;
  VertexOut.depth         = depth;
  VertexOut.lateral_quality = bf_result.y;
  gl_Position             = gua_projection_matrix * gua_view_matrix * gua_model_matrix * vec4(VertexOut.pos_cs, 1.0);
}

