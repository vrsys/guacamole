// clamp splat size to max. surfel size
float splat_size = radius_scaling * in_radius;

// scale tangent to desired splat size
VertexOut.ms_u = ms_u * splat_size;
VertexOut.ms_v = ms_v * splat_size;
  
