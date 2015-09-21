@include "common/header.glsl"

// uniforms
uniform int light_id;

layout(binding=0, r32ui) uniform coherent uimage3D light_bitset;

void main()
{
  // calculate slice in light_bitset (a 3D-Texture)
  // divide by 32
  ivec3 pos = ivec3(gl_FragCoord.xy, light_id >> 5);

#if @light_table_fallback_mode@
  pos.xy = pos.xy >> @light_table_tile_power@;
#endif

  // determine responsible bit for this light
  uint bit = 1u << (light_id % 32);
  // add visible light to light_bitset
  imageAtomicOr(light_bitset, pos, bit);
}

