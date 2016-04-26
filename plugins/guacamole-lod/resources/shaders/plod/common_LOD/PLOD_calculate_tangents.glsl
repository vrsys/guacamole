  vec3 ms_n = normalize(in_normal.xyz);

  vec3 ms_u;
  //compute u and v vectors
  if(ms_n.z != 0.0) {
    ms_u = vec3(1, 1, (-ms_n.x -ms_n.y)/ms_n.z);
  }
  else if(ms_n.y != 0.0) {
    ms_u = vec3(1, (-ms_n.x -ms_n.z)/ms_n.y, 1);
  }
  else {
    ms_u = vec3( (-ms_n.y -ms_n.z)/ms_n.x , 1, 1);
  }
