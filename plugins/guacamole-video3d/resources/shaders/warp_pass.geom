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
    float lateral_quality;
} VertexIn[3];


///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
out vec2  texture_coord;
out vec3  pos_es;
out vec3  pos_cs;
out float sq_area_cs;

out float depth;
out float lateral_quality;
out vec3  normal_es;


///////////////////////////////////////////////////////////////////////////////
// methods 
///////////////////////////////////////////////////////////////////////////////
bool validSurface(vec3 a, vec3 b, vec3 c,
                  float depth_a, float depth_b, float depth_c)
{
  if(depth_a < 0.1 || depth_b < 0.1 || depth_c < 0.1) {
    return false;
  }

  float avg_depth = (depth_a + depth_b + depth_c)/3.0;
  float baselength = 0.005;
  float l = 0.0125 * avg_depth + baselength;  
  // compare squared distance to save sqrt
  l *= l;
  return ((dot(a, a) < l) && (dot(b, b) < l) && (dot(c, c) < l));
}

float calcAreaSQ(vec3 a, vec3 b, vec3 c){
  vec3 ab = b - a;
  vec3 ac = c - a;
  vec3 cc = cross(ab,ac);
  return dot(cc, cc);
}


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main()
{
  float depth_a = VertexIn[0].depth;
  float depth_b = VertexIn[1].depth;
  float depth_c = VertexIn[2].depth;

  vec3 a_cs = VertexIn[1].pos_cs - VertexIn[0].pos_cs;
  vec3 b_cs = VertexIn[2].pos_cs - VertexIn[0].pos_cs;
  vec3 c_cs = VertexIn[2].pos_cs - VertexIn[1].pos_cs;

  bool valid = validSurface(a_cs, b_cs, c_cs, depth_a, depth_b, depth_c);

  if (valid)
  {
    vec3 a = VertexIn[1].pos_es - VertexIn[0].pos_es;
    vec3 b = VertexIn[2].pos_es - VertexIn[0].pos_es;
    vec3 c = VertexIn[2].pos_es - VertexIn[1].pos_es;

    vec3 tri_normal = normalize(cross (a, b));
  #if 0
    // flip normal towards camera in eye space. However, this will be done in fragment stage
    if ( dot ( tri_normal, -normalize(VertexIn[0].pos_es) ) < 0.0f ) {
      tri_normal = -tri_normal;
    }
  #endif

    const float sq_area_in_cs  = calcAreaSQ(VertexIn[0].pos_cs,VertexIn[1].pos_cs,VertexIn[2].pos_cs);
      
    for(int i = 0; i < gl_in.length(); i++)
    {
      texture_coord = VertexIn[i].texture_coord;
      pos_es        = VertexIn[i].pos_es;
      pos_cs        = VertexIn[i].pos_cs;
      sq_area_cs    = sq_area_in_cs;
      lateral_quality = VertexIn[i].lateral_quality;
      depth         = VertexIn[i].depth;
      normal_es     = tri_normal;

      gl_Position   = gl_in[i].gl_Position;

      EmitVertex();
    }
    EndPrimitive();
  }
}
