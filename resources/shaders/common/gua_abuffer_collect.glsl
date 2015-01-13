
#if @enable_abuffer@

@include "gua_abuffer.glsl"

void abuf_insert(float depth)
{
  const ivec2 frag_pos = ivec2(gl_FragCoord.xy);

  uint ctr = atomicCounterIncrement(frag_counter);
  float frag_alpha = clamp(gua_alpha, 0.0, 1.0);

  uint record_location = ctr + abuf_list_offset;
  frag_list[record_location] = 0ul;
  memoryBarrier();

  // pack depth and alpha
  uint z_ordered = bitfieldInsert(pack_depth24(depth),
                                  uint(round(frag_alpha * 256.0)), 0, 8);

  uint64_t old, record = packUint2x32(uvec2(record_location, z_ordered));

  uint pos = gua_resolution.x * frag_pos.y + frag_pos.x; // start of the search
  int frag_ctr = 0;
  float accum_alpha = 0;
  bool success = false;

  uint64_t assumed;

  // insert record to the linked-list
  while (true) {
#ifdef GL_NV_shader_atomic_int64
    old = atomicMax(frag_list[pos], record);
#else
    old = frag_list[pos];
    do {
      assumed = old;
      old = atomicCompSwap(frag_list[pos], assumed, MAX64(record, assumed));
    } while (assumed != old);
#endif
    if (old == 0) {
      success = true;
      break;
    } else {
      if (old > record) { // go to next
        pos = LSB64(old);
        // early termination
        if (!success) {
          float current_frag_alpha = float(bitfieldExtract(unpackUint2x32(old).y, 0, 8)) / 256.0;
          accum_alpha += mix(current_frag_alpha, 0.0, accum_alpha);
          if (accum_alpha >= @abuf_blending_termination_threshold@) {
            break;
          }
        }
      } 
      else { // inserted
        pos = LSB64(record);
        record = old;
        success = true;
      }
    }
    if (frag_ctr++ >= ABUF_MAX_FRAGMENTS) break; 
  }

  if (success) {
    // write data
    ABUF_FRAG(ctr, 0) = vec4(gua_color, frag_alpha);
    ABUF_FRAG(ctr, 1) = vec4(gua_emissivity, gua_roughness, gua_metalness,
                             uintBitsToFloat((gua_flags_passthrough)?1:0));
    ABUF_FRAG(ctr, 2) = vec4(fma(gua_normal, vec3(0.5), vec3(0.5)), 0);
  }
}

#endif

void submit_fragment(float depth)
{
#if @enable_abuffer@
  if (gua_alpha > @abuf_insertion_threshold@) {
    @include "gua_write_gbuffer.glsl"
  }
  else {
    abuf_insert(depth);
    discard;
  }
#else
    @include "gua_write_gbuffer.glsl"
#endif
}

