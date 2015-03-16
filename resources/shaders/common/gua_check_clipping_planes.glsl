void check_clipping_planes() {

  for (int i=0; i < gua_clipping_plane_count; ++i) {

    if (dot(gua_clipping_planes[i].xyz, gua_position) + gua_clipping_planes[i].w < 0) {
      discard;
    }
  }
}

