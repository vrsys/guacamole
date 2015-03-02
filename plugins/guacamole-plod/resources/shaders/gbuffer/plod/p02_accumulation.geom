@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

uniform bool enable_backface_culling;
uniform mat4 model_view_matrix;
uniform mat4 model_view_projection_matrix;

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VertexData {
  //input from geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;

  vec3 pass_point_color;
  vec3 pass_normal;
} VertexIn[];

//out VertexData {
out vec3 pass_point_color;
out vec3 pass_normal;
out vec2 pass_uv_coords;

//} VertexOut;
@include "common/gua_vertex_shader_output.glsl"
 

float index_arr[8] = {-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

void main() {

    if(enable_backface_culling == false /*|| VertexIn[0].pass_normal.z > 0.0*/) {

      // --------------------------- common attributes -----------------------------------
      pass_point_color = VertexIn[0].pass_point_color;
      pass_normal = VertexIn[0].pass_normal; 

      vec3 s_pos_ms = gl_in[0].gl_Position.xyz; // poisition of surfel in model space
      vec3 step_u   = VertexIn[0].pass_ms_u;
      vec3 step_v   = VertexIn[0].pass_ms_v;

      float es_linear_depth_center = (model_view_matrix * vec4(s_pos_ms,1.0)).z;
      float es_shift = 0.0;
      float es_shift_scale = 2.0;


      // ---------------------------------------------------------------------------------
      for(int idx = 0; idx < 4; ++idx ) {
        float u_multiplier = index_arr[idx];
        float v_multiplier = index_arr[idx + 4];

        pass_uv_coords        = vec2(u_multiplier, v_multiplier);
        vec4 q_pos_ms         = vec4( ( (s_pos_ms + (u_multiplier * step_u) ) + (v_multiplier * step_v) ) ,1.0);
        gl_Position           = model_view_projection_matrix * q_pos_ms;
        gua_varying_position  = (gua_model_matrix * q_pos_ms).xyz;
        float es_linear_depth_corner = (model_view_matrix * q_pos_ms).z;

        es_shift       = abs(es_linear_depth_corner - es_linear_depth_center) * es_shift_scale;
        gl_Position.z  = ( ( -(es_linear_depth_corner + es_shift ) ) / gua_clip_far);
        gl_Position.z  = (gl_Position.z - 0.5) * 2.0;
        gl_Position.z  *= gl_Position.w;

        EmitVertex();
      }

      EndPrimitive();
      
  }

}

