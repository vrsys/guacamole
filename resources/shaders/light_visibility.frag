@include "common/header.glsl"


layout(binding=0, r32ui) uniform coherent uimage3D light_bitset;

in flat uint light_id_shift_5;
in flat uint light_bit;
void main()
{
  // calculate slice in light_bitset (a 3D-Texture)
  // divide by 32
  ivec3 pos = ivec3(gl_FragCoord.xy, light_id_shift_5);

#if @light_table_fallback_mode@
  pos.xy = pos.xy >> @light_table_tile_power@;
#endif

  // determine responsible bit for this light
  //uint bit = 1u << (light_id_mod_32);
  // add visible light to light_bitset
  imageAtomicOr(light_bitset, pos, light_bit);
}

