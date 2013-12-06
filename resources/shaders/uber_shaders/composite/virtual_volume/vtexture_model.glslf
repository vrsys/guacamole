
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#version 420 core

#extension GL_ARB_shading_language_include : require

#include </scm/data/vtexture/vtexture.glslh>
#include </scm/data/vtexture/vtexture_debug.glslh>

// input layout definitions ///////////////////////////////////////////////////////////////////////
layout(early_fragment_tests) in;

// input/output definitions ///////////////////////////////////////////////////////////////////////
in per_vertex {
    vec3 normal;
    vec2 texcoord;
    vec3 view_dir;
} v_in;

// attribute layout definitions ///////////////////////////////////////////////////////////////////
layout(location = 0, index = 0) out vec4 out_color;

// uniform input definitions //////////////////////////////////////////////////////////////////////
uniform float       fixed_lod;
uniform vtexture2D  vtex_color;

// subroutine declaration
subroutine vec4 generate_color();
subroutine uniform generate_color out_color_gen;

// global constants ///////////////////////////////////////////////////////////////////////////////
const int   max_binary_search_steps = 10;
const float epsilon                 = 0.0001;

const vec3 white = vec3(1.0, 1.0, 1.0);
const vec3 black = vec3(0.0, 0.0, 0.0);
const vec3 red   = vec3(1.0, 0.0, 0.0);
const vec3 green = vec3(0.0, 1.0, 0.0);
const vec3 blue  = vec3(0.0, 0.0, 1.0);
const vec3 lblue = vec3(0.2, 0.7, 0.9);

// subroutines
subroutine (generate_color)
vec4 visualize_normal()
{
    return vec4(v_in.normal, 1.0);
}

subroutine (generate_color)
vec4 visualize_texcoord()
{
    return vec4(v_in.texcoord, 0.0, 1.0);
}

subroutine (generate_color)
vec4 draw_vtexture()
{
    return vtexture(vtex_color, v_in.texcoord);
}

subroutine (generate_color)
vec4 draw_vtexture_lod()
{
    return vtexture_lod(vtex_color, v_in.texcoord, fixed_lod);
}

subroutine (generate_color)
vec4 draw_debug_atlas()
{
    return vtexture_debug_atlas(vtex_color, v_in.texcoord);
}

subroutine (generate_color)
vec4 draw_debug_page_trilinear()
{
    return vtexture_debug_page_trilinear(vtex_color, v_in.texcoord);
}

subroutine (generate_color)
vec4 draw_debug_quadtree()
{
    return vtexture_debug_quadtree(vtex_color, v_in.texcoord);
}

// implementation /////////////////////////////////////////////////////////////////////////////////
void main() 
{
#if VTEX_USE_FBACK_IMAGE == 1
    vtexture_require(vtex_color, v_in.texcoord);
#endif
#if VTEX_USE_FBACK_LISTS == 1
    vtexture_add_request(vtex_color, v_in.texcoord);
#endif
    //ivec2 frag_coord = ivec2(gl_FragCoord.xy);
    //ivec2 out_coord = frag_coord / _Vtex_FB_capture_factor;
    //memoryBarrier();
    //vec4 c = vec4(bitfieldReverse(imageLoad(_Vtex_FB_data_image, out_coord)) >> 24u) * (vec4(1.0) / vec4(255.0));

    vec4 c = out_color_gen();//draw_vtexture();//draw_debug_quadtree();//
    //vec4 c =draw_debug_quadtree();//out_color_gen();//draw_debug_quadtree();//
    //vec4 c = vtexture(vtex_color, v_in.texcoord);//out_color_gen();//vec4(v_in.normal, 1.0);
    //vec4 c = vtexture_debug_page_trilinear(vtex_color, v_in.texcoord);//out_color_gen();//vec4(v_in.normal, 1.0);
    //vec4 c = vtexture_debug_quadtree(vtex_color, v_in.texcoord);//out_color_gen();//vec4(v_in.normal, 1.0);
    //vec4 c = draw_vtexture(vtex_color, v_in.texcoord).rrrr;//out_color_gen();//vec4(v_in.normal, 1.0);

#if 0
    vec3 n = normalize(v_in.normal); 
    vec3 l = normalize(vec3(1.0, 1.0, 1.0)); // assume parallel light!
    vec3 v = normalize(v_in.view_dir);
    vec3 h = normalize(l + v);

    out_color.rgb =    c.rgb * (dot(n, l) * 0.5 + 0.5)//max(0.0, dot(n, l))
                     + lblue * pow(max(0.0, dot(n, h)), 120.0);
#else
    out_color.rgb =  c.rgb;
#endif
    out_color.a   = 0.3;
}

