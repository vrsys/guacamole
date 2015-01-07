float edge_length(in vec3 v1,
  in vec3 v2,
  in mat4 mvp_matrix,
  in int screen_resolution_x,
  in int screen_resolution_y)
{
  vec4 cs_v1 = mvp_matrix * vec4(v1, 1.0);
  vec4 cs_v2 = mvp_matrix * vec4(v2, 1.0);

  cs_v1 = cs_v1 / cs_v1.w;
  cs_v2 = cs_v2 / cs_v2.w;

  vec2 dv1 = (cs_v1.xy * 0.5 + 0.5) * vec2(screen_resolution_x, screen_resolution_y);
  vec2 dv2 = (cs_v2.xy * 0.5 + 0.5) * vec2(screen_resolution_x, screen_resolution_y);

  vec2 edge = dv1.xy - dv2.xy;

  return clamp(length(edge), 0, max(screen_resolution_x, screen_resolution_y));
}
