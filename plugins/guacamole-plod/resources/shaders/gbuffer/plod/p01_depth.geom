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


float offset_depth_value(float in_gl_Pos_z) {

  //in_gl_Pos_z /= in_gl_Pos_w; //perspective division
  //in_gl_Pos_z = (in_gl_Pos_z * 0.5) + 0.5; //[-1,1]^2 -> [0,1]^2

  //linearize the log depth
  float log_z = in_gl_Pos_z;
  float lin_z = (2.0 * gua_clip_near * gua_clip_far) / (gua_clip_far + gua_clip_near - log_z * (gua_clip_far - gua_clip_near));

  //add some offset here
  //...

  float A = gua_projection_matrix[2].z;
  float B = gua_projection_matrix[3].z;

  //logarithmize the offset lin depth
  //log_z = ( ((2.0 * gua_clip_near)/lin_z) - (gua_clip_far + gua_clip_far) ) / (gua_clip_far - gua_clip_near);

  log_z = 0.5 * (-A * lin_z + B) / lin_z + 0.5;

  //undo viewport transformation
  log_z = (log_z - 0.5) * 2.0;
  //log_z *= in_gl_Pos_w;

  return log_z;

}
 
void main() {

  mat4 MV = gua_view_matrix * gua_model_matrix;
  mat4 MVP = gua_projection_matrix * gua_view_matrix * gua_model_matrix;

  gl_Position = MVP * vec4( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u + VertexIn[0].pass_ms_v), 1.0);
  VertexOut.pass_uv_coords = vec2(1.0, 1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0)).z;

  gl_Position /= gl_Position.w;
  gl_Position.z = offset_depth_value(gl_Position.z);
  EmitVertex();

  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0);
  VertexOut.pass_uv_coords = vec2(1.0, -1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;

  gl_Position /= gl_Position.w;
  gl_Position.z = offset_depth_value(gl_Position.z);
  EmitVertex();

  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0);
  VertexOut.pass_uv_coords = vec2(-1.0, 1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0) ).z;

  gl_Position /= gl_Position.w;
  gl_Position.z = offset_depth_value(gl_Position.z);
  EmitVertex();

  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz - VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0);
  VertexOut.pass_uv_coords = vec2(-1.0, -1.0);
  VertexOut.pass_log_depth = (gl_Position.z/gl_Position.w)/2 + 0.5;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz - VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;

  gl_Position /= gl_Position.w;
  gl_Position.z = offset_depth_value(gl_Position.z);
  EmitVertex();


  EndPrimitive();


}

