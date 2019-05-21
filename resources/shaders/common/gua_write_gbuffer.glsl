gua_out_color     = gua_color;

gua_out_pbr       = vec3(gua_emissivity, gua_roughness, gua_metalness);
gua_out_normal    = gua_normal*0.5+0.5;

if(gua_flags_passthrough) {
	gua_out_flags = 1u;
}

