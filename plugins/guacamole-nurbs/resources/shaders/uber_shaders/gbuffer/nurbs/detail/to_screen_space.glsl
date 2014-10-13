vec4 to_screen_space(in vec3 point,
                     in mat4 mvp_matrix,
                     in int screen_res_x,
                     in int screen_res_y)
{
  vec4 ret_point;

  ret_point = mvp_matrix * vec4(point, 1.0);
  ret_point = ret_point / ret_point.w;
  ret_point = vec4((ret_point.xy * 0.5 + 0.5) * vec2(screen_res_x, screen_res_y), 1.0, 1.0);

  return ret_point;
}