bool is_inside(vec4 point)
{
  if ((point.x >= -1.0 && point.x <= 1.0) && (point.y >= -1.0 && point.y <= 1.0) && (point.z >= -1.0 && point.z <= 1.0)) {
    return true;
  }

  return false;
}