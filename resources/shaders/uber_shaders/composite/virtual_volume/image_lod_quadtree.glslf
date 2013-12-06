
#version 330 core

#extension GL_ARB_shading_language_include : require

#include </scm/large_data/virtual_texture/vtexture.glsl>
#include </scm/large_data/virtual_texture/vtexture_quadtree.glsl>

// input/output definitions ///////////////////////////////////////////////////////////////////////
in per_vertex {
    vec2 virtual_texcoord;
} v_in;

// attribute layout definitions ///////////////////////////////////////////////////////////////////
layout(location = 0, index = 0) out vec4 out_color;

// uniform input definitions //////////////////////////////////////////////////////////////////////
uniform vtexture2D  vtex_large_image;

vec4
vtexture_pagecoords_trilinear(in vtexture2D vtex,       // virtual texture decriptor struct
                              in vec2       vtex_coord) // virtual texture coordinate
{
    // calculate the required lod
    float target_level      = vtex.max_level;

    vec2  dummy_ptex_coord_low;
    vec2  dummy_ptex_coord_high;
    vec2  dummy_page_coord_trilinear_low;
    vec2  page_coord_trilinear_high;
    float dummt_page_coord_trilinear_blend;

    quadtree_traverse(vtex, vtex_coord, target_level,
                      dummy_page_coord_trilinear_low, page_coord_trilinear_high,
                      dummy_ptex_coord_low, dummy_ptex_coord_high,
                      dummt_page_coord_trilinear_blend);

    return vec4(page_coord_trilinear_high, 0.0, 1.0);
}

void main() 
{
    out_color = vtexture_pagecoords_trilinear(vtex_large_image, v_in.virtual_texcoord);
}

