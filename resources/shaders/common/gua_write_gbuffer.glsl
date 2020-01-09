gua_out_color     = gua_color;

gua_out_pbr       = vec3(gua_emissivity, gua_roughness, gua_metalness);

if(gua_emissivity < 1.0) { 
	gua_out_normal = gua_normal*0.5+0.5;
}

gua_out_flags = gua_flags_passthrough ? 1u : 0;


