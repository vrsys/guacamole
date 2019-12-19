// clamp splat size to max. surfel size
//float splat_size = clamp(radius_scaling * in_radius, 0.0, max_surfel_radius);

float splat_size = radius_scaling * in_radius;

// scale tangent to desired splat size
VertexOut.pass_ms_u = ms_u * splat_size;
VertexOut.pass_ms_v = ms_v * splat_size;
  
