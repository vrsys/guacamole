struct per_patch_data
{
  uint surface_offset;
  uint16_t order_u;
  uint16_t order_v;
  uint trim_id;
  uint obb_id;

  vec4 nurbs_domain;
  vec4 bbox_min;
  vec4 bbox_max;

  vec4 dist;
};

layout(std430, binding = GPUCAST_ATTRIBUTE_SSBO_BINDING) buffer attribute_buffer{
  per_patch_data attribute_data[];
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void retrieve_patch_order(in int index, out int order_u, out int order_v) 
{
  order_u = clamp(int(attribute_data[index].order_u), 1, 10);
  order_v = clamp(int(attribute_data[index].order_v), 1, 10);
}

///////////////////////////////////////////////////////////////////////////////
void retrieve_patch_data(in int index, out int point_index, out int order_u, out int order_v)
{
  point_index = int(attribute_data[index].surface_offset);
  order_u = int(attribute_data[index].order_u);
  order_v = int(attribute_data[index].order_v);
}

///////////////////////////////////////////////////////////////////////////////
int retrieve_controlpoint_index(in int index)
{
  return int(attribute_data[index].surface_offset);
}

///////////////////////////////////////////////////////////////////////////////
int retrieve_trim_index(in int index)
{
  return int(attribute_data[index].trim_id);
}

///////////////////////////////////////////////////////////////////////////////
int retrieve_obb_index(in int index)
{
  return int(attribute_data[index].obb_id);
}

///////////////////////////////////////////////////////////////////////////////
vec4 retrieve_patch_domain(in int index)
{
  return attribute_data[index].nurbs_domain;
}

///////////////////////////////////////////////////////////////////////////////
void retrieve_patch_bbox(in int index, out vec4 bboxmin, out vec4 bboxmax) 
{
  bboxmin = attribute_data[index].bbox_min;
  bboxmax = attribute_data[index].bbox_max;
}

///////////////////////////////////////////////////////////////////////////////
vec4 retrieve_patch_distance(in int index)
{
  return attribute_data[index].dist;
}
