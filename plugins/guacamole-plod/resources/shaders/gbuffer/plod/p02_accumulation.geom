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

  vec3 pass_point_color;
  vec3 pass_normal;
} VertexIn[];

//out VertexData {
out vec3 pass_point_color;
out vec3 pass_normal;
out vec2 pass_uv_coords;

//} VertexOut;
@include "common/gua_vertex_shader_output.glsl"
 

void main() {

    if(enable_backface_culling == false || VertexIn[0].pass_normal.z > 0.0) {

      mat4 MV = gua_view_matrix * gua_model_matrix;
      mat4 MVP = gua_projection_matrix * gua_view_matrix * gua_model_matrix;

      // --------------------------- common attributes -----------------------------------
      pass_point_color = VertexIn[0].pass_point_color;
      pass_normal = VertexIn[0].pass_normal;

      vec3 s_pos_ms = gl_in[0].gl_Position.xyz; // poisition of surfel in model space
      vec3 step_u   = VertexIn[0].pass_ms_u;
      vec3 step_v   = VertexIn[0].pass_ms_v;

      float es_linear_depth = (MV * vec4(s_pos_ms,1.0)).z;
      float es_shift = 0.0;
      float es_shift_scale = 1.0;

      // --------------------------- (-1.0,-1.0) -----------------------------------------
      pass_uv_coords = vec2(-1.0, -1.0);
      vec4 q_pos_msA =  vec4( ( (s_pos_ms - step_u) - step_v ), 1.0);
      gl_Position            = MVP * q_pos_msA;
      gua_varying_position   = (gua_model_matrix * q_pos_msA).xyz;
      float es_linear_depthA = (MV * q_pos_msA).z;

      es_shift = abs(es_linear_depthA - es_linear_depth) * es_shift_scale;
      gl_Position.z = ( ( -(es_linear_depthA + es_shift ) ) / gua_clip_far);
      gl_Position.z = (gl_Position.z - 0.5) * 2.0; 
      gl_Position.z *= gl_Position.w;

      EmitVertex();

      // --------------------------- (-1.0, 1.0) -----------------------------------------
      pass_uv_coords = vec2(-1.0, 1.0);
      vec4 q_pos_msB =  vec4( ( (s_pos_ms - step_u) + step_v ), 1.0);
      gl_Position            = MVP * q_pos_msB;
      gua_varying_position   = (gua_model_matrix * q_pos_msB).xyz;
      float es_linear_depthB = (MV * q_pos_msB).z;
      
      es_shift = abs(es_linear_depthB - es_linear_depth) * es_shift_scale;
      gl_Position.z = ( ( -(es_linear_depthB + es_shift ) ) / gua_clip_far);
      gl_Position.z = (gl_Position.z - 0.5) * 2.0; 
      gl_Position.z *= gl_Position.w;

      EmitVertex();
      // --------------------------- (1.0, -1.0) -----------------------------------------
      pass_uv_coords = vec2(1.0, -1.0);
      vec4 q_pos_msC =  vec4( ( (s_pos_ms + step_u) - step_v ), 1.0);
      gl_Position            = MVP * q_pos_msC;
      gua_varying_position   = (gua_model_matrix * q_pos_msC).xyz;
      float es_linear_depthC = (MV * q_pos_msC).z;
      
      es_shift = abs(es_linear_depthC - es_linear_depth) * es_shift_scale;
      gl_Position.z = ( ( -(es_linear_depthC + es_shift ) ) / gua_clip_far);
      gl_Position.z = (gl_Position.z - 0.5) * 2.0; 
      gl_Position.z *= gl_Position.w;

      EmitVertex();
      // --------------------------- (1.0, 1.0) -----------------------------------------
      pass_uv_coords = vec2(1.0, 1.0);
      vec4 q_pos_msD =  vec4( ( (s_pos_ms + step_u) + step_v ), 1.0);
      gl_Position            = MVP * q_pos_msD;
      gua_varying_position   = (gua_model_matrix * q_pos_msD).xyz;
      float es_linear_depthD = (MV * q_pos_msD).z;
      
      es_shift = abs(es_linear_depthD - es_linear_depth) * es_shift_scale;
      gl_Position.z = ( ( -(es_linear_depthD + es_shift ) ) / gua_clip_far);
      gl_Position.z = (gl_Position.z - 0.5) * 2.0; 
      gl_Position.z *= gl_Position.w;

      EmitVertex();

      // --------------------------- END of QUAD   -----------------------------------------
      EndPrimitive();

#if 0    
      ws_pos = gua_model_matrix * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0);
      gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v ), 1.0);
      gua_varying_position = ws_pos.xyz;

    /*VertexOut.*/pass_uv_coords = vec2(1.0, -1.0);

    /*VertexOut.*/pass_point_color = VertexIn[0].pass_point_color;
    /*VertexOut.*/pass_normal = VertexIn[0].pass_normal;

    /*VertexOut.*/es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;


    //gl_Position.z = ( ( -(pass_es_linear_depth+0.01) )/ gua_clip_far);
    gl_Position.z = ( ( -(es_linear_depth+es_rad ) )/ gua_clip_far);
    gl_Position.z = (gl_Position.z - 0.5) * 2.0; 

    gl_Position.z *= gl_Position.w;
    EmitVertex();

    //gl_Position = VP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0);
    ws_pos = gua_model_matrix * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0);
    gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u ), 1.0);
    gua_varying_position = ws_pos.xyz;
    /*VertexOut.*/pass_uv_coords = vec2(-1.0, 1.0);

    /*VertexOut.*/pass_point_color = VertexIn[0].pass_point_color;
    /*VertexOut.*/pass_normal = VertexIn[0].pass_normal;

    /*VertexOut.*/es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) - VertexIn[0].pass_ms_u), 1.0) ).z;


    //gl_Position.z = ( ( -(pass_es_linear_depth+0.01) )/ gua_clip_far);
    gl_Position.z = ( ( -(es_linear_depth+es_rad ) )/ gua_clip_far);
    
    gl_Position.z = (gl_Position.z - 0.5) * 2.0; 

    gl_Position.z *= gl_Position.w;
    EmitVertex();

    //gl_Position = VP * vec4( ( (gl_in[0].gl_Position.xyz - VertexIn[0].pass_ms_u ) - VertexIn[0].pass_ms_v), 1.0);
    ws_pos = gua_model_matrix * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0);
    gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz - VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v ), 1.0);
    gua_varying_position = ws_pos.xyz;
    /*VertexOut.*/pass_uv_coords = vec2(-1.0, -1.0);

    /*VertexOut.*/pass_point_color = VertexIn[0].pass_point_color;
    /*VertexOut.*/pass_normal = VertexIn[0].pass_normal;

    /*VertexOut.*/es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz - VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0) ).z;

    gl_Position.z = ( ( -(es_linear_depth+es_rad ) )/ gua_clip_far);
    gl_Position.z = (gl_Position.z - 0.5) * 2.0; 
    gl_Position.z *= gl_Position.w;

    EmitVertex();



    EndPrimitive();

#endif

  }

}

