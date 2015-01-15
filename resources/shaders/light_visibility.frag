@include "common/header.glsl"

// uniforms
uniform int light_id;

layout(binding=0, r32ui) uniform coherent uimage3D light_bitset;

void main()
{
  ivec3 pos = ivec3(gl_FragCoord.xy, light_id >> 5);

#if @light_table_fallback_mode@
  pos.xy = pos.xy >> @light_table_tile_power@;
#endif

  uint bit = 1u << (light_id % 32);
  imageAtomicOr(light_bitset, pos, bit);
}

