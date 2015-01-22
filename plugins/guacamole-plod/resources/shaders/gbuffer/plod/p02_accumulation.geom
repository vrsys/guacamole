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

  vec3 pass_point_color;
  vec3 pass_normal;
  float pass_es_radius;
} VertexIn[];

out VertexData {
  vec3 pass_point_color;
  vec3 pass_normal;
  vec2 pass_uv_coords;
  float pass_es_linear_depth;
  float pass_ms_rad;
} VertexOut;

 
void main() {

  mat4 MV = gua_view_matrix * gua_model_matrix;
  mat4 MVP = gua_projection_matrix * gua_view_matrix * gua_model_matrix;



  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0);
  gl_Position.z = (gl_Position.z - 0.1 );

// gl_Position.z = 
  VertexOut.pass_uv_coords = vec2(1.0, 1.0);

  VertexOut.pass_point_color = VertexIn[0].pass_point_color;
  VertexOut.pass_normal = VertexIn[0].pass_normal;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0)).z;
  VertexOut.pass_ms_rad = length(VertexIn[0].pass_ms_u);
  EmitVertex();

  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0);
  gl_Position.z = (gl_Position.z - 0.1 );
  VertexOut.pass_uv_coords = vec2(1.0, -1.0);

  VertexOut.pass_point_color = VertexIn[0].pass_point_color;
  VertexOut.pass_normal = VertexIn[0].pass_normal;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;
  VertexOut.pass_ms_rad = length(VertexIn[0].pass_ms_u);
  EmitVertex();

  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0);
  gl_Position.z = (gl_Position.z - 0.1 );
  VertexOut.pass_uv_coords = vec2(-1.0, 1.0);

  VertexOut.pass_point_color = VertexIn[0].pass_point_color;
  VertexOut.pass_normal = VertexIn[0].pass_normal;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0) ).z;
  VertexOut.pass_ms_rad = length(VertexIn[0].pass_ms_u);
  EmitVertex();

  gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz - VertexIn[0].pass_ms_u ) - VertexIn[0].pass_ms_v), 1.0);
  gl_Position.z = (gl_Position.z - 0.1 );
  VertexOut.pass_uv_coords = vec2(-1.0, -1.0);

  VertexOut.pass_point_color = VertexIn[0].pass_point_color;
  VertexOut.pass_normal = VertexIn[0].pass_normal;

  VertexOut.pass_es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz - VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;
  VertexOut.pass_ms_rad = length(VertexIn[0].pass_ms_u);
  EmitVertex();



  EndPrimitive();


}

