vec4 control_polygon_length(in samplerBuffer	data,
  in mat4           mvp_matrix,
  in int		        offset,
  in int 		        u,
  in int		        v,
  in int            screen_res_x,
  in int            screen_res_y)
{
  int i, j;
  vec4 result = vec4(0.0);

  //      3                                                                                                                                     
  //	3------2                                                                                                                                  
  //	|      |                                                                                                                                  
  //0 |      | 2                                                                                                                                
  //  |      |                                                                                                                                  
  //  0------1                                                                                                                                  
  //      1                                                                                                                                     

  /*For Edge 03*/
  for (i = u; i < u * v - u + 1; i += u) {
    result[0] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - u).xyz, mvp_matrix, screen_res_x, screen_res_y);
  }

  /*For Edge 01*/
  for (i = 1; i < u; ++i) {
    result[1] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - 1).xyz, mvp_matrix, screen_res_x, screen_res_y);
  }

  /*For Edge 12*/
  for (i = 2 * u - 1; i < u * v; i += u) {
    result[2] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - u).xyz, mvp_matrix, screen_res_x, screen_res_y);
  }

  /*For Edge 23*/
  for (i = u * v - u + 1; i < u * v; ++i) {
    result[3] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - 1).xyz, mvp_matrix, screen_res_x, screen_res_y);
  }

  return result;
}