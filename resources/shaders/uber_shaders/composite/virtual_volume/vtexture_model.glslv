
#version 420 core

#extension GL_ARB_shading_language_include : require

#include </scm/gl_util/camera_block.glslh>

precision highp float;

// attribute layout definitions ///////////////////////////////////////////////////////////////////
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_texture_coord;

// input/output definitions ///////////////////////////////////////////////////////////////////////
out per_vertex {
    vec3 normal;
    vec2 texcoord;
    vec3 view_dir;
} v_out;

// uniform input definitions //////////////////////////////////////////////////////////////////////
uniform mat4 mv_matrix;
uniform mat4 mv_matrix_it;

// implementation /////////////////////////////////////////////////////////////////////////////////
void main()
{
    //v_out.view_dir = -normalize(camera_transform.v_matrix * vec4(in_position, 1.0)).xyz;
    //v_out.texcoord = in_texture_coord;
    //v_out.normal   = (camera_transform.v_matrix_inverse_transpose * vec4(in_normal, 0.0)).xyz;
    //gl_Position    = camera_transform.vp_matrix * vec4(in_position, 1.0);
    v_out.view_dir = -normalize(mv_matrix * vec4(in_position, 1.0)).xyz;
    v_out.texcoord = in_texture_coord;
    v_out.normal   = (mv_matrix_it * vec4(in_normal, 0.0)).xyz;
    gl_Position    = camera_transform.p_matrix * mv_matrix * vec4(in_position, 1.0);
}
