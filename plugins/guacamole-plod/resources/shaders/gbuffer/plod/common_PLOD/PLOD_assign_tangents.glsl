  vec3 tmp_ms_v       = normalize( cross(ms_n,ms_u) );
  VertexOut.pass_ms_u = normalize( cross(tmp_ms_v,ms_n) ) * radius_importance_scaling  * 2.0 * in_radius;
  VertexOut.pass_ms_v = tmp_ms_v                          * radius_importance_scaling  * 2.0 * in_radius;