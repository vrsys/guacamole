
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#version 420 core
#extension GL_EXT_gpu_shader5 : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_texture_array : enable


out VertexData {
    vec3 normal;
    vec2 texture_coord;
    vec3 view_dir;
} VertexOut;

layout (binding = 1) uniform sampler2DArray kinect_depths;

uniform mat4 image_d_to_eye_d;
uniform mat4 eye_d_to_world;
uniform mat4 eye_d_to_eye_rgb;
uniform mat4 eye_rgb_to_image_rgb;

uniform int layer;

out vec2 tex_coord;
out vec3 pos_es;
out vec3 pos_d;
out vec3 pos_ws;

out float depth;

uniform mat4 projection_matrix;
uniform mat4 model_view_matrix;
uniform mat4 model_view_matrix_inverse_transpose;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_texture_coord;

void main()
{
    depth = texture2DArray(kinect_depths, vec3(in_texture_coord.xy, layer)).r;

    vec4 POS_d = depth * image_d_to_eye_d * vec4(in_position.xy, depth, 1.0);
    POS_d.z = depth;

    POS_d.w = 1.0;
    pos_d   = POS_d.xyz;

    vec4 POS_rgb = eye_d_to_eye_rgb * POS_d;

    if(POS_rgb.z > 0.0)
        VertexOut.texture_coord = (eye_rgb_to_image_rgb * vec4( (POS_rgb.xy/POS_rgb.z) ,0.0,1.0)).xy;
    else
        VertexOut.texture_coord = vec2(0.0);

    vec4 POS_ws =  eye_d_to_world * POS_d;

    if(POS_ws.y < 0.0)
    POS_ws.y = 0.0;

    gl_Position = projection_matrix * model_view_matrix * POS_ws;
    pos_es = (model_view_matrix * POS_ws).xyz;
    pos_ws = POS_ws.xyz;
/////////////////////////////////////

    VertexOut.normal        =  normalize(model_view_matrix_inverse_transpose * vec4(in_normal, 0.0)).xyz;
    VertexOut.view_dir      = -normalize(model_view_matrix * vec4(in_position, 1.0)).xyz;
    //VertexOut.texture_coord = in_texture_coord;

    //gl_Position = projection_matrix * model_view_matrix * vec4(in_position, 1.0);
}
