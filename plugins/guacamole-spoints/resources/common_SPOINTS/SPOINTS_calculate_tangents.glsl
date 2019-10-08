  // normal should be normalized already!
  vec3 ms_n = in_normal.xyz; // normalize(in_normal.xyz);

  #if 1
  // compute vector perpendicular to normal : ms_u 
  vec3 ms_u;

  if(ms_n.z != 0.0) {
    ms_u = vec3(1, 1, (-ms_n.x -ms_n.y)/ms_n.z);
  }
  else if(ms_n.y != 0.0) {
    ms_u = vec3(1, (-ms_n.x -ms_n.z)/ms_n.y, 1);
  }
  else {
    ms_u = vec3( (-ms_n.y -ms_n.z)/ms_n.x , 1, 1);
  }
  #else 
    vec3 ms_u = vec3((-ms_n.y-ms_n.y) * ms_n.z, ms_n.z * ms_n.x, ms_n.y * ms_n.x);
  #endif

  // re-normalize first tangent
  ms_u = normalize(ms_u);

  // find vector perpendicular to normal and tangent ms_u
  vec3 ms_v = cross(ms_n, ms_u);