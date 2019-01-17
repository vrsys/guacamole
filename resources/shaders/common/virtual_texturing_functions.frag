
uniform int gua_current_vt_idx;

struct physical_texture_info_struct {
  uvec2 address;
  uvec2 tile_size;
  vec2  tile_padding;
  uvec2 dims;
};

//layout(std140, binding = 8) uniform physical_texture_address { physical_texture_info_struct pt; };


float dxdy()
{
    vec2 c = gua_uvs.xy * 256;

    vec2 dFdxCxy = dFdx(c);
    vec2 dFdyCxy = dFdy(c);

    float rho = sqrt(max( dFdxCxy.x*dFdxCxy.x + dFdxCxy.y*dFdxCxy.y, 
                          dFdyCxy.x*dFdyCxy.x + dFdyCxy.y*dFdyCxy.y ) );

    float lambda = log2(rho);

    return lambda;
}

void writeVTCoords(int vt_idx = gua_current_vt_idx) {

  float lambda = -dxdy();

  gua_uvs.z = lambda;
  gua_uvs.w = vt_idx;

  vec3 uv_lambda_triple  = vec3(gua_uvs.xy, lambda);
  vec2 sampled_uv_coords = uv_lambda_triple.rg;

  sampled_uv_coords.y = 1.0 - sampled_uv_coords.y;
}