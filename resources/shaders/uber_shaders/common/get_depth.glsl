float gua_get_depth(vec2 frag_pos) {
    return texture2D(gua_get_float_sampler(gua_depth_gbuffer_in), frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth() {
    vec2 frag_pos = gua_get_quad_coords();
    return texture2D(gua_get_float_sampler(gua_depth_gbuffer_in), frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth(sampler2D depth_texture, vec2 frag_pos) {
    return texture2D(depth_texture, frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth(sampler2D depth_texture) {
    vec2 frag_pos = gua_get_quad_coords();
    return texture2D(depth_texture, frag_pos).x * 2.0 - 1.0;
}
