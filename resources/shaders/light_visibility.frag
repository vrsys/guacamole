@include "common/header.glsl"

// uniforms
@include "common/gua_camera_uniforms.glsl"
uniform int light_id;

layout(binding=0, r32ui) uniform coherent uimage3D light_bitset;

void main()
{
  ivec3 pos = ivec3(gl_FragCoord.xy, light_id >> 5);
  //pos.xy = pos.xy >> @light_table_tile_power@;

  uint bit = 1u << (light_id % 32);

  if ((imageLoad(light_bitset, pos).x & bit) == 0) {
    imageAtomicOr(light_bitset, pos, bit);
  }

  discard;
}

