float edge_tesslevel(in float length,
  in float max_error)
{
  return clamp(length / max_error, 1.0f, 64.0f);
}