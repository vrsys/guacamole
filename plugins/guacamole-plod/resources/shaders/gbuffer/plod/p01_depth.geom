@include "resources/shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "resources/shaders/common/gua_camera_uniforms.glsl"


layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VertexData {
  //input from geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_ms_center;
} VertexIn[];

out VertexData {
  vec2 pass_uv_coords;
  float pass_log_depth;
  float pass_es_linear_depth;
} VertexOut;

 
void main() {

  mat4 MV = gua_view_matrix * gua_model_matrix;
  mat4 MVP = gua_projection_matrix * MV;

  gl_Position = MVP * vec4( (VertexIn[0].pass_ms_center + VertexIn[0].pass_ms_u + VertexIn[0].pass_ms_v), 1.0);
  VertexOut.pass_uv_coords = vec2(1.0, 1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (VertexIn[0].pass_ms_center + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0)).z;
  EmitVertex();

  gl_Position = MVP * vec4( ( (VertexIn[0].pass_ms_center + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0);
  VertexOut.pass_uv_coords = vec2(1.0, -1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (VertexIn[0].pass_ms_center + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;
  EmitVertex();

  gl_Position = MVP * vec4( ( (VertexIn[0].pass_ms_center + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0);
  VertexOut.pass_uv_coords = vec2(-1.0, 1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (VertexIn[0].pass_ms_center + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0) ).z;
  EmitVertex();

  gl_Position = MVP * vec4( ( (VertexIn[0].pass_ms_center - VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0);
  VertexOut.pass_uv_coords = vec2(-1.0, -1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (VertexIn[0].pass_ms_center - VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;
  EmitVertex();


  EndPrimitive();


}

