@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"


layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VertexData {
  //input from geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  //vec3 pass_ms_center;
} VertexIn[];

out VertexData {
  vec2 pass_uv_coords;
  float pass_log_depth;
  float pass_es_linear_depth;
} VertexOut;


 
void main() {

  mat4 MV = gua_view_matrix * gua_model_matrix;
  mat4 MVP = gua_projection_matrix * gua_view_matrix * gua_model_matrix;

  gl_Position = MVP * vec4( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u + VertexIn[0].pass_ms_v), 1.0);
  VertexOut.pass_uv_coords = vec2(1.0, 1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  float es_linear_depth= (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0)).z;



  //gl_Position /= gl_Position.w;
  gl_Position.z = (-(es_linear_depth+0.01) / gua_clip_far);
  
  gl_Position.z = (gl_Position.z - 0.5) * 2.0; 

  gl_Position.z *= gl_Position.w;

  VertexOut.pass_es_linear_depth  = (-es_linear_depth / gua_clip_far);

  EmitVertex();

  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0);
  VertexOut.pass_uv_coords = vec2(1.0, -1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;

  //gl_Position /= gl_Position.w;
  gl_Position.z = (-(es_linear_depth+0.01) / gua_clip_far);
  
  gl_Position.z = (gl_Position.z - 0.5) * 2.0; 

  gl_Position.z *= gl_Position.w;

  VertexOut.pass_es_linear_depth  = (-es_linear_depth / gua_clip_far);
  EmitVertex();

  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0);
  VertexOut.pass_uv_coords = vec2(-1.0, 1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0) ).z;

  //gl_Position /= gl_Position.w;
  gl_Position.z = (-(es_linear_depth+0.01) / gua_clip_far);
  
  gl_Position.z = (gl_Position.z - 0.5) * 2.0; 

  gl_Position.z *= gl_Position.w;

  VertexOut.pass_es_linear_depth  = (-es_linear_depth / gua_clip_far);
  EmitVertex();

  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz - VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0);
  VertexOut.pass_uv_coords = vec2(-1.0, -1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz - VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;

  //gl_Position /= gl_Position.w;
  gl_Position.z = (-(es_linear_depth+0.01) / gua_clip_far);
  
  gl_Position.z = (gl_Position.z - 0.5) * 2.0; 

  gl_Position.z *= gl_Position.w;

  VertexOut.pass_es_linear_depth  = (-es_linear_depth / gua_clip_far);
  EmitVertex();


  EndPrimitive();


}

