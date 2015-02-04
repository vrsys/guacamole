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

    //float es_rad = 0.1;

    vec4 ws_pos = gua_model_matrix * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0);
    gl_Position = MVP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0);
    gua_varying_position = ws_pos.xyz;
    // gl_Position.z = 
    /*VertexOut.*/pass_uv_coords = vec2(1.0, 1.0);

    /*VertexOut.*/pass_point_color = VertexIn[0].pass_point_color;
    /*VertexOut.*/pass_normal = VertexIn[0].pass_normal;

    float es_linear_depth = (MV * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_v) + VertexIn[0].pass_ms_u ), 1.0)).z;


    float es_rad = 0.01;
/*
     if(pass_es_linear_depth > -1) {
       es_rad = 0.001;
     }
     else {
      es_rad = 0.1;
     }
*/
    //gl_Position.z = ( ( -(pass_es_linear_depth+0.01) )/ gua_clip_far);
    gl_Position.z = ( ( -(es_linear_depth+es_rad ) )/ gua_clip_far);
    gl_Position.z = (gl_Position.z - 0.5) * 2.0; 

    gl_Position.z *= gl_Position.w;
    EmitVertex();

    //gl_Position = VP * vec4( ( (gl_in[0].gl_Position.xyz + VertexIn[0].pass_ms_u) - VertexIn[0].pass_ms_v), 1.0);
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


    //gl_Position.z = ( ( -(pass_es_linear_depth+0.01) )/ gua_clip_far);
    gl_Position.z = ( ( -(es_linear_depth+es_rad ) )/ gua_clip_far);
    
    gl_Position.z = (gl_Position.z - 0.5) * 2.0; 

    gl_Position.z *= gl_Position.w;
    EmitVertex();



    EndPrimitive();

  }

}

