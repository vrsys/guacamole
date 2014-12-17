gua_out_color     = gua_color;
gua_out_pbr       = vec4(gua_emissivity, gua_roughness, gua_metalness, (gua_passthrough_shading)?1.0:-1.0);
gua_out_normal    = gua_normal*0.5+0.5;
