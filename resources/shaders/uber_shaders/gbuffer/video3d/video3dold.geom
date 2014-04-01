#version 420 core

#extension GL_EXT_gpu_shader4 : enable
#extension GL_EXT_geometry_shader4 : enable

layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;

uniform float min_length;
uniform float geo_length;

in VertexData {
    vec3 normal;
    vec2 texture_coord;
    vec3 view_dir;
} VertexIn[3];

out VertexData {
    vec3 normal;
    vec2 texture_coord;
    vec3 view_dir;
} VertexOut;

in vec3 pos_es[];
in vec3 pos_d[];
in float depth[];

bool validSurface(vec3 a, vec3 b, vec3 c)
{
  float avg_depth = (depth[0] + depth[1] + depth[2])/3.0;
  float l = min_length * avg_depth + geo_length;
  if((length(a) > l) || (length(b) > l) || (length(c) > l)){
    return false;
  }
  return true;
}

vec3 computeNormal(vec3 x1, vec3 x2, vec3 x3)
{
    vec3 norm = cross(x3 - x1, x2 - x1);
    return normalize(norm);
}

void main()
{
  vec3 a = pos_es[1] - pos_es[0];
  vec3 b = pos_es[2] - pos_es[0];
  vec3 c = pos_es[2] - pos_es[1];

  bool valid = validSurface(a,b,c);
  if (valid)
  {
      vec3 normal = computeNormal(gl_in[0].gl_Position.xyz, gl_in[1].gl_Position.xyz, gl_in[2].gl_Position.xyz);
      for(int i = 0; i < gl_in.length(); i++)
      {
        // copy attributes
        gl_Position = gl_in[i].gl_Position;
        //VertexOut.normal = VertexIn[i].normal;
        VertexOut.normal = normal;
        VertexOut.texture_coord = VertexIn[i].texture_coord;
        VertexOut.view_dir = VertexIn[i].view_dir;

        // done with the vertex
        EmitVertex();
      }
      EndPrimitive();
  }
}
