@include "common/header.glsl"

// uniforms
uniform int light_id;

layout(binding=0, r32ui) uniform coherent uimage3D light_bitset;

#if @get_enable_multi_view_rendering@
layout(binding=1, r32ui) uniform coherent uimage3D secondary_light_bitset;
#endif

#if @get_enable_multi_view_rendering@
in flat int is_for_right_eye;
#endif
out vec4 out_Color;
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


#if @get_enable_multi_view_rendering@
  if(0 == is_for_right_eye) {
#endif
  	imageAtomicOr(light_bitset, pos, bit);
#if @get_enable_multi_view_rendering@
  } else {
 	  imageAtomicOr(secondary_light_bitset, pos, bit);
  }
#endif 

out_Color = vec4(float(bit)*255.0, 0.0, 0.0, 1.0);
}

