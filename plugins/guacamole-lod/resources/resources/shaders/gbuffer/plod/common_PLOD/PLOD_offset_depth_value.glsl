float offset_depth_value(float in_gl_Pos_z) {

  //in_gl_Pos_z /= in_gl_Pos_w; //perspective division
  //in_gl_Pos_z = (in_gl_Pos_z * 0.5) + 0.5; //[-1,1]^2 -> [0,1]^2

  //linearize the log depth
  float log_z = in_gl_Pos_z;
  float lin_z = (2.0 * gua_clip_near * gua_clip_far) / (gua_clip_far + gua_clip_near - log_z * (gua_clip_far - gua_clip_near));

  //add some offset here
  //...

  float A = gua_projection_matrix[2].z;
  float B = gua_projection_matrix[3].z;

  //logarithmize the offset lin depth
  //log_z = ( ((2.0 * gua_clip_near)/lin_z) - (gua_clip_far + gua_clip_far) ) / (gua_clip_far - gua_clip_near);

  log_z = 0.5 * (-A * lin_z + B) / lin_z + 0.5;

  //undo viewport transformation
  log_z = (log_z - 0.5) * 2.0;
  //log_z *= in_gl_Pos_w;

  return log_z;

}
