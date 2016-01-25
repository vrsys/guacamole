  //steppo
  //vec3 tmp_ms_v       = normalize( cross(ms_n,ms_u) );
  //VertexOut.pass_ms_u = normalize( cross(tmp_ms_v,ms_n) ) * radius_importance_scaling  * 2.0 * in_radius;
  //VertexOut.pass_ms_v = normalize(tmp_ms_v) * 2.0 * in_radius ;

  //adrian
VertexOut.pass_ms_u = normalize(ms_u) * radius_scaling * in_radius;
VertexOut.pass_ms_v = normalize(cross(ms_n, ms_u)) * radius_scaling  * in_radius;
  
if (in_radius > 0.1) {
    VertexOut.pass_ms_u = vec3(0);
    VertexOut.pass_ms_v = vec3(0);
}