bool frustum_cull(in mat4 mvp_matrix)
{
  vec4 cs_v1 = mvp_matrix * vec4(vertex_position[0], 1.0);
  vec4 cs_v2 = mvp_matrix * vec4(vertex_position[1], 1.0);
  vec4 cs_v3 = mvp_matrix * vec4(vertex_position[2], 1.0);
  vec4 cs_v4 = mvp_matrix * vec4(vertex_position[3], 1.0);

  cs_v1 = cs_v1 / cs_v1.w;
  cs_v2 = cs_v2 / cs_v2.w;
  cs_v3 = cs_v3 / cs_v3.w;
  cs_v4 = cs_v4 / cs_v4.w;

  if (is_inside(cs_v1) || is_inside(cs_v2) || is_inside(cs_v3) || is_inside(cs_v4))
  {
    return true;
  }

  return false;
}