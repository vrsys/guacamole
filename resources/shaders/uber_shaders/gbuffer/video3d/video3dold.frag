
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#version 420 core

#extension GL_EXT_gpu_shader5 : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_EXT_texture_array : enable

in VertexData {
    vec3 normal;
    vec2 texture_coord;
    vec3 view_dir;
} VertexIn;

uniform vec3    light_ambient;
uniform vec3    light_diffuse;
uniform vec3    light_specular;
uniform vec3    light_position;
uniform vec3    kColorIntesity;

uniform int layer;

uniform vec3    material_ambient;
uniform vec3    material_diffuse;
uniform vec3    material_specular;
uniform float   material_shininess;
uniform float   material_opacity;

layout (binding = 0) uniform sampler2DArray kinect_colors;
layout (binding = 2) uniform sampler2D sample_colors;


layout(location = 0) out vec4        out_color;

void main()
{
    vec4 res;
    vec3 n = normalize(VertexIn.normal);
    vec3 l = normalize(light_position); // assume parallel light!
    vec3 v = normalize(VertexIn.view_dir);
    vec3 h = normalize(l + v);//gl_LightSource[0].halfVector);//

    vec4 txlC = texture(sample_colors, VertexIn.texture_coord);
    vec4 txlK = texture2DArray(kinect_colors, vec3(VertexIn.texture_coord.xy, layer) );

    vec4 txl = vec4( kColorIntesity.r * txlK.r + (1-kColorIntesity.r) * txlC.r,
                kColorIntesity.g * txlK.g + (1-kColorIntesity.g) * txlC.g,
                kColorIntesity.b * txlK.b + (1-kColorIntesity.b) * txlC.b,
                txlC.a);

    //vec4 txl = texture(kinect_depths, VertexIn.texture_coord);

    if(light_position != vec3(0.0, 0.0, 0.0)){
        res.rgb =  light_ambient * material_ambient
             + light_diffuse * txl.rgb * max(0.0, dot(n, l))
             + light_specular * material_specular * pow(max(0.0, dot(n, h)), material_shininess);
        res.a = material_opacity;
    }
    else{
        res = txl;
    }

    out_color = res;
}

