#ifdef GL_NV_shader_atomic_int64
#extension GL_NV_shader_atomic_int64 : enable
#endif

void abuf_insert()
{
  const ivec2 frag_pos = ivec2(gl_FragCoord.xy);

  uint ctr = atomicCounterIncrement(frag_counter);

  // write data
  ABUF_FRAG(ctr, 0) = vec4(gua_color, gua_alpha);
  ABUF_FRAG(ctr, 1) = vec4(gua_emissivity, gua_roughness, gua_metalness, 0);
  ABUF_FRAG(ctr, 2) = vec4(gua_normal*0.5+0.5, 0);

  // insert to the linked-list
  ctr += abuf_list_offset;
  frag_list[ctr] = 0ul;
  memoryBarrier();

  float z = LIN_DEPTH(gl_FragCoord.z);
  uint64_t old, record = packUint2x32(uvec2(ctr, PACK_DEPTH(z)));

  uint pos = gua_resolution.x * frag_pos.y + frag_pos.x; // start of the search
  int frag_ctr = 0;
  bool stop_flag = false;
  float alpha = 0;

  uint64_t assumed;

  while (!stop_flag) {
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
      stop_flag = true;
    } else {
      if (old > record) { // go to next
        pos = LSB64(old);
      } 
      else { // inserted
        pos = LSB64(record);
        record = old;
      }//*/
      //pos = LSB64(MAX64(old, record));
      //record = MIN64(old, record);
    }
    if (frag_ctr++ >= ABUF_MAX_FRAGMENTS) break; 
  }

}

