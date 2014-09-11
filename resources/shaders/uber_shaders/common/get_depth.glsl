float gua_get_depth(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth() {
    vec2 frag_pos = gua_get_quad_coords();
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth(sampler2D depth_texture, vec2 frag_pos) {
    return texture2D(depth_texture, frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth(sampler2D depth_texture) {
    vec2 frag_pos = gua_get_quad_coords();
    return texture2D(depth_texture, frag_pos).x * 2.0 - 1.0;
}
