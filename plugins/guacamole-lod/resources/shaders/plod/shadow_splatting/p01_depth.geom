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
  vec3 pass_world_position;
} VertexOut;



void main() {

  if(enable_backface_culling == false || VertexIn[0].pass_normal.z > 0.0) {

      // --------------------------- common attributes -----------------------------------
      mat3x3 step_uv = mat3x3(gl_in[0].gl_Position.xyz,
                              VertexIn[0].pass_ms_u,
                              VertexIn[0].pass_ms_v);

      const float index_arr[8] = {-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

      // ---------------------------------------------------------------------------------
      for(int idx = 0; idx < 4; ++idx ) {
        vec3 uv_multiplier = vec3(1.0, 
                                  index_arr[idx],   
                                  index_arr[idx + 4]);

        VertexOut.pass_uv_coords        = uv_multiplier.yz;
        vec4 q_pos_ms                   = vec4( (step_uv * uv_multiplier) , 1.0);
        gl_Position                     = gua_model_view_projection_matrix * q_pos_ms;
        VertexOut.pass_world_position   = (gua_model_matrix * q_pos_ms).xyz;
        EmitVertex();
      }

      EndPrimitive();
  }

}

