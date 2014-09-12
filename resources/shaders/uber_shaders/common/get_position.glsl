vec3 gua_get_position(vec2 frag_pos) {
    vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, gua_get_depth(frag_pos), 1.0);
    vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
    h /= h.w; 
    return h.xyz;
} 
 
vec3 gua_get_position() { 
    return gua_get_position(gua_get_quad_coords());
}

vec3 gua_get_position(sampler2D depth_texture, vec2 frag_pos) {
    vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, gua_get_depth(depth_texture, frag_pos), 1.0);
    vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
    h /= h.w;
    return h.xyz;
}

vec3 gua_get_position(sampler2D depth_texture) {
    return gua_get_position(depth_texture, gua_get_quad_coords());
}

