@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

uniform bool enable_backface_culling;

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VertexData {
  //input from geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_normal;
} VertexIn[];

out VertexData {
  vec2 pass_uv_coords;
  float pass_log_depth;
  float pass_es_linear_depth;
} VertexOut;


float index_arr[8] = {-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

 
void main() {

  if(enable_backface_culling == false /*|| VertexIn[0].pass_normal.z > 0.0*/ ) {
    mat4 MV = gua_view_matrix * gua_model_matrix;
    mat4 MVP = gua_projection_matrix * gua_view_matrix * gua_model_matrix;


      // --------------------------- common attributes -----------------------------------

      vec3 s_pos_ms = gl_in[0].gl_Position.xyz; // poisition of surfel in model space
      vec3 step_u   = VertexIn[0].pass_ms_u;
      vec3 step_v   = VertexIn[0].pass_ms_v;

      float es_linear_depth_center = (MV * vec4(s_pos_ms,1.0)).z;
      float es_shift = 0.0;
      float es_shift_scale = 2.0;


      // ---------------------------------------------------------------------------------
      for(int idx = 0; idx < 4; ++idx ) {
        float u_multiplier = index_arr[idx];
        float v_multiplier = index_arr[idx + 4];

        VertexOut.pass_uv_coords        = vec2(u_multiplier, v_multiplier);
        vec4 q_pos_ms                   = vec4( ( (s_pos_ms + (u_multiplier * step_u) ) + (v_multiplier * step_v) ) ,1.0);
        gl_Position                     = MVP * q_pos_ms;
        VertexOut.pass_log_depth        = (gl_Position.z/gl_Position.w)/2.0 + 0.5;

        float es_linear_depth_corner = (MV * q_pos_ms).z;

        es_shift       = abs(es_linear_depth_corner - es_linear_depth_center) * es_shift_scale;
        gl_Position.z  = ( ( -(es_linear_depth_corner /*+ es_shift*/ ) ) / gua_clip_far);
        gl_Position.z  = (gl_Position.z - 0.5) * 2.0;
        gl_Position.z  *= gl_Position.w;

        VertexOut.pass_es_linear_depth = (-es_linear_depth_corner) / gua_clip_far;

        EmitVertex();
      }

      EndPrimitive();
  }

}

