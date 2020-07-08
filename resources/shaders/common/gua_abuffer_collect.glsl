@include "gua_check_clipping_planes.glsl"

#if @enable_abuffer@

uniform uvec2 gua_gbuffer_depth;

@include "gua_abuffer.glsl"

bool abuf_insert(float depth)
{
  const ivec2 frag_pos = ivec2(gl_FragCoord.xy);

  uint ctr = atomicCounterIncrement(frag_counter);
  float frag_alpha = clamp(gua_alpha, 0.0, 1.0);

  uint record_location = ctr + abuf_list_offset;
  frag_list[record_location] = 0ul;
  memoryBarrier();

  // pack depth and alpha
  uint z_ordered = bitfieldInsert(pack_depth24(depth),
                                  uint(round(frag_alpha * 255.0)), 0, 8);

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
          float current_frag_alpha = float(bitfieldExtract(unpackUint2x32(old).y, 0, 8)) / 255.0;
          accum_alpha += mix(current_frag_alpha, 0.0, accum_alpha);
          if (accum_alpha > @abuf_blending_termination_threshold@) {
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

    uint pbr = packUnorm4x8(vec4(gua_emissivity, gua_roughness, gua_metalness, 0.0));
    pbr = bitfieldInsert(pbr, ((gua_flags_passthrough)?1u:0u), 24, 8);

    uint col_norm = bitfieldInsert(packUnorm2x16(gua_color.bb),
                                   packSnorm2x16(gua_normal.xx), 16, 16);

    frag_data[ctr] = uvec4(packUnorm2x16(gua_color.rg), col_norm,
                           packSnorm2x16(gua_normal.yz), pbr);
  }
  return success;
}

#endif

void submit_fragment(float depth)
{
  check_clipping_planes();

  // if abuffer enabled and not rendering shadows
  if ((bool)@enable_abuffer@ && gua_rendering_mode == 0) {
#if @enable_abuffer@

  #if @get_enable_multi_view_rendering@
    float z = texelFetch(sampler2DArray(gua_gbuffer_depth), ivec3(gl_FragCoord.xy, gl_ViewID_OVR), 0).x;
  #else
    float z = texelFetch(sampler2D(gua_gbuffer_depth), ivec2(gl_FragCoord.xy), 0).x;
  #endif
    if (depth > z) discard;

    if (gua_alpha < 1.0 - @abuf_insertion_threshold@) {
      discard;
    }

    if (gua_alpha > @abuf_insertion_threshold@) {
      @include "gua_write_gbuffer.glsl"
    }
    else {
      if (abuf_insert(depth))
        discard;
    }
#endif
  } 
  else {
    if (gua_alpha < 0.5) {
      discard;
    }
    @include "gua_write_gbuffer.glsl"
  }
}

