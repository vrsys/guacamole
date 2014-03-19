isampler2D gua_get_int_sampler(uvec2 handle) {
    return isampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler2D gua_get_uint_sampler(uvec2 handle) {
    return usampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_float_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_double_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2DShadow  gua_get_shadow_sampler(uvec2 handle) {
    return sampler2DShadow(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

isampler3D gua_get_int_sampler3D(uvec2 handle) {
    return isampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler3D gua_get_uint_sampler3D(uvec2 handle) {
    return usampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_float_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_double_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}
