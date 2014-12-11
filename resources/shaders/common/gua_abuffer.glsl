
// HINT: define ABUF_MODE before including this file to change memory  
//       qualifier of frg_list and frg_storage buffers
#ifndef ABUF_MODE 
#define ABUF_MODE
#endif


#define ABUF_STORE_OFFSET 3
#define ABUF_MAX_FRAGMENTS 300
#define ABUF_CULL 1
#define ABUF_CULL_THRES 0.96

// If max for uint64_t is not available
#if 1
#define MAX64(x, y) (((x)>(y))?(x):(y))
#define MIN64(x, y) (((x)<(y))?(x):(y))
#else
// this is not available with old drivers
#define MAX64(x, y) max((x), (y))
#define MIN64(x, y) min((x), (y))
#endif

uniform ivec2 gua_resolution;

// buffers
layout (binding = 0) uniform atomic_uint frag_counter;
layout (std430, binding = 0) ABUF_MODE coherent buffer abuf_list {
  uint64_t frag_list[];
};

layout (std430, binding = 1) ABUF_MODE coherent buffer abuf_data {
  vec4 frag_data[];
};

// helper macros
#define UINT_MAX             4294967295U
#define ABUF_FRAG(i,j)       (frag_data[(i)*ABUF_STORE_OFFSET+(j)])
#define LSB64(a)             (uint32_t(a))
#define LIN_DEPTH(z)         ((2*gua_clip_near) / (gua_clip_far+gua_clip_near - (z) * (gua_clip_far-gua_clip_near)))
#define UNLIN_DEPTH(z)       (((z) * (gua_clip_far+gua_clip_near) - 2*gua_clip_near) / ((z) * (gua_clip_far-gua_clip_near)))
#define PACK_DEPTH(a)        (uint(float(1.0-(a))*float(UINT_MAX)))
#define UNPACK_DEPTH(a)      (float(1.0-float(a)/float(UINT_MAX)))

const uint abuf_list_offset = gua_resolution.x * gua_resolution.y;

