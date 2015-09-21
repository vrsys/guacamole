float inner_tess_level(in samplerBuffer data,
  in int offset,
  in mat4 mvp_matrix,
  in float max_error,
  in int screen_res_x,
  in int screen_res_y)
{
  vec4 bbox_min = texelFetch(data, offset + 2);
  vec4 bbox_max = texelFetch(data, offset + 3);
  vec3 combin[8];
  int i, j;
  float max_length = 0.0;

  combin[0] = to_screen_space(vec3(bbox_min.x, bbox_min.y, bbox_min.z), mvp_matrix, screen_res_x, screen_res_y).xyz;
  combin[1] = to_screen_space(vec3(bbox_min.x, bbox_min.y, bbox_max.z), mvp_matrix, screen_res_x, screen_res_y).xyz;
  combin[2] = to_screen_space(vec3(bbox_min.x, bbox_max.y, bbox_min.z), mvp_matrix, screen_res_x, screen_res_y).xyz;
  combin[3] = to_screen_space(vec3(bbox_min.x, bbox_max.y, bbox_max.z), mvp_matrix, screen_res_x, screen_res_y).xyz;
  combin[4] = to_screen_space(vec3(bbox_max.x, bbox_min.y, bbox_min.z), mvp_matrix, screen_res_x, screen_res_y).xyz;
  combin[5] = to_screen_space(vec3(bbox_max.x, bbox_min.y, bbox_max.z), mvp_matrix, screen_res_x, screen_res_y).xyz;
  combin[6] = to_screen_space(vec3(bbox_max.x, bbox_max.y, bbox_min.z), mvp_matrix, screen_res_x, screen_res_y).xyz;
  combin[7] = to_screen_space(vec3(bbox_max.x, bbox_max.y, bbox_max.z), mvp_matrix, screen_res_x, screen_res_y).xyz;

  for (i = 0; i < 7; i++)
  {
    for (j = i + 1; j < 8; j++)
    {
      float temp_length = length(combin[i].xy - combin[j].xy);

      if (temp_length > max_length)
      {
        max_length = temp_length;
      }
    }
  }

  max_length = clamp(max_length, 0, max(screen_res_x, screen_res_y));

  return clamp(ceil(max_length / max_error), 2.0, 64.0);
}