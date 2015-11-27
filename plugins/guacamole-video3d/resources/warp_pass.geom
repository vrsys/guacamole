@include "shaders/common/header.glsl"

layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;


///////////////////////////////////////////////////////////////////////////////
// constants
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// video3d uniforms
///////////////////////////////////////////////////////////////////////////////
uniform int layer;

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in VertexData {
    vec2 texture_coord;
    vec3 pos_es;
    vec3 pos_cs;
    float depth;
} VertexIn[3];


///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
out vec2  texture_coord;
out vec3  pos_es;
out vec3  pos_cs;
out float depth;
out vec3  normal_es;


///////////////////////////////////////////////////////////////////////////////
// methods 
///////////////////////////////////////////////////////////////////////////////
bool validSurface(vec3 a, vec3 b, vec3 c,
                  float depth_a, float depth_b, float depth_c)
{
  float avg_depth = (depth_a + depth_b + depth_c)/3.0;
  float baselength = 0.005;
  float l = 0.0125 * avg_depth + baselength;	

  if((length(a) > l) || (length(b) > l) || (length(c) > l)){
    return false;
  }

  if(depth_a < 0.1 || depth_b < 0.1 || depth_c < 0.1)
  {
        return false;
  }

  return true;
}




///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main()
{
  vec3 a = VertexIn[1].pos_es - VertexIn[0].pos_es;
  vec3 b = VertexIn[2].pos_es - VertexIn[0].pos_es;
  vec3 c = VertexIn[2].pos_es - VertexIn[1].pos_es;

  vec3 tri_normal = normalize(cross (a, b));
  if ( dot ( tri_normal, -normalize(VertexIn[0].pos_es) ) < 0.0f ) {
    tri_normal = -tri_normal;
  }

  float depth_a = VertexIn[0].depth;
  float depth_b = VertexIn[1].depth;
  float depth_c = VertexIn[2].depth;

  vec3 a_cs = VertexIn[1].pos_cs - VertexIn[0].pos_cs;
  vec3 b_cs = VertexIn[2].pos_cs - VertexIn[0].pos_cs;
  vec3 c_cs = VertexIn[2].pos_cs - VertexIn[1].pos_cs;

  bool valid = validSurface(a_cs, b_cs, c_cs, depth_a, depth_b, depth_c);

  if (valid)
  {      
      for(int i = 0; i < gl_in.length(); i++)
      {
        texture_coord = VertexIn[i].texture_coord;
        pos_es        = VertexIn[i].pos_es;
        pos_cs        = VertexIn[i].pos_cs;
        depth         = VertexIn[i].depth;
        normal_es     = tri_normal;

        gl_Position   = gl_in[i].gl_Position;

        EmitVertex();
      }
      EndPrimitive();
  }
}
