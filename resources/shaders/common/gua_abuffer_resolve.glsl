void abuf_mix_frag(vec4 frag_color, inout vec4 color) {
  frag_color.rgb *= frag_color.a;
  color += mix(frag_color, vec4(0.0), color.a);
}

#if @enable_abuffer@

@include "gua_abuffer.glsl"

#ifndef ABUF_SHADE_FUNC
#error "ABUF_SHADE_FUNC has not been defined!"
#else
vec4 ABUF_SHADE_FUNC(uint pos, float depth);
#endif

bool abuf_blend(inout vec4 color, inout float emissivity, float opaque_depth) {
  const ivec2 frag_pos = ivec2(gl_FragCoord.xy);
  uint current = gua_resolution.x * frag_pos.y + frag_pos.x;
  int frag_count = 0;

  while (frag_count < ABUF_MAX_FRAGMENTS) {

    uvec2 frag = unpackUint2x32(frag_list[current]);
    if (frag.x == 0) {
      break;
    }
    ++frag_count;

    float z = unpack_depth24(frag.y);
    vec4 shaded_color = ABUF_SHADE_FUNC(frag.x - abuf_list_offset, fma(z, 2.0, -1.0));

    if (gua_compositing_enable) {
      if (z + 0.000001 > opaque_depth) { // fix depth-fighting artifacts
        break;
      }
      #if @gua_write_abuffer_depth@
        if (frag_count == 1) {
          gl_FragDepth = z;
        }
      #endif
      float frag_alpha = float(bitfieldExtract(frag.y, 0, 8)) / 255.0;
      emissivity = min(1.0, emissivity + (1-color.a)*shaded_color.w*frag_alpha);
      abuf_mix_frag(vec4(shaded_color.rgb, frag_alpha), color);
      if (color.a > @abuf_blending_termination_threshold@) {
        return false;
      }
    } else {
      if (z + 0.000001 > opaque_depth) {
        frag_list[current] = 0;
        break;
      } else {
        frag_data[frag.x - abuf_list_offset].rgb = floatBitsToUint(shaded_color.rgb);
      }
    }

    current = frag.x;
  }
  return true;
}
#endif

