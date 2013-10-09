uint gua_get_material_id() {
    return texture2D(gua_get_uint_sampler(gua_uint_gbuffer_in_1[0]), gua_get_quad_coords()).x;
}
