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
  flat uint pass_global_surfel_id;
} VertexIn[];

out VertexData {
  vec2 pass_uv_coords;
  float es_depth;
  flat uint pass_further_global_surfel_id;
  vec3 pass_world_position;
} VertexOut;



void main() {

  //if(enable_backface_culling == false) {

      // --------------------------- common attributes -----------------------------------

      mat3x3 step_uv = mat3x3(gl_in[0].gl_Position.xyz,
                              VertexIn[0].pass_ms_u,
                              VertexIn[0].pass_ms_v);

      float es_linear_depth_center = (gua_model_view_matrix * vec4(step_uv[0],1.0)).z;
      float es_shift = 0.0;
      float es_shift_scale = 2.0;

      const float index_arr[8] = {-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

      // ---------------------------------------------------------------------------------
      for(int idx = 0; idx < 4; ++idx ) {
        vec3 uv_multiplier = vec3(1.0, 
                                  index_arr[idx],   
                                  index_arr[idx + 4]);

        vec4 q_pos_ms                   = vec4( (step_uv * uv_multiplier) , 1.0);
        gl_Position                     = gua_model_view_projection_matrix * q_pos_ms;
        float es_linear_depth_corner    = (gua_model_view_matrix * q_pos_ms).z;

        VertexOut.pass_uv_coords                = uv_multiplier.yz;
        VertexOut.es_depth                      = -es_linear_depth_corner;
        VertexOut.pass_further_global_surfel_id = VertexIn[0].pass_global_surfel_id;
        VertexOut.pass_world_position           = (gua_model_matrix * q_pos_ms).xyz;

        EmitVertex();
      }

      EndPrimitive();
  //}

}

