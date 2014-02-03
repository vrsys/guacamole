// header
@include "shaders/common/header.glsl"
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_texture_array : enable

layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;

// material specific uniforms
@uniform_definition

// per-vertex output
@output_definition

// global variables
vec2 gua_texcoords;

vec3 gua_world_normal;
vec3 gua_world_position;
vec3 gua_world_tangent;
vec3 gua_world_bitangent;

vec3 gua_object_normal;
vec3 gua_object_position;
vec2 gua_object_texcoords;
vec3 gua_object_tangent;
vec3 gua_object_bitangent;

out vec3 gua_position_varying;

float min_length = 0.0125; //TODO uniform
float geo_length = 0.01; //TODO uniform

// uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

uniform uint gua_material_id;

//global gua methods
@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

// material
uint gua_get_material_id() {
  return gua_material_id;
}

//material methods
@material_methods

in VertexData {
    vec3 normal; // delete
    vec2 texture_coord;
    vec3 view_dir; // delete
    vec3 pos_object_space;
    vec3 pos_eye_space;
    vec3 pos_d; // delete
    float depth;
} VertexIn[3];

bool validSurface(vec3 a, vec3 b, vec3 c)
{
  float avg_depth = (VertexIn[0].depth + VertexIn[1].depth + VertexIn[2].depth)/3.0;
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
  vec3 a = VertexIn[1].pos_eye_space - VertexIn[0].pos_eye_space;
  vec3 b = VertexIn[2].pos_eye_space - VertexIn[0].pos_eye_space;
  vec3 c = VertexIn[2].pos_eye_space - VertexIn[1].pos_eye_space;

  bool valid = validSurface(a,b,c);
  if (valid)
  {
      vec3 normal = computeNormal(gl_in[0].gl_Position.xyz, gl_in[1].gl_Position.xyz, gl_in[2].gl_Position.xyz);
      for(int i = 0; i < gl_in.length(); i++)
      {
        // copy attributes
        // gl_Position = gl_in[i].gl_Position;

        gua_position_varying = vec3(0);

        gua_texcoords = VertexIn[i].texture_coord;

        gua_object_normal =     normal;
        gua_object_tangent =    a;
        gua_object_bitangent =  b;
        gua_object_position =   VertexIn[i].pos_object_space;

        gua_world_normal =      normalize((gua_normal_matrix * vec4(normal, 0.0)).xyz);
        gua_world_tangent =     normalize((gua_normal_matrix * vec4(a, 0.0)).xyz);
        gua_world_bitangent =   normalize((gua_normal_matrix * vec4(b, 0.0)).xyz);
        gua_world_position =    (gua_model_matrix * vec4(VertexIn[i].pos_object_space, 1.0)).xyz;

        // big switch, one case for each material
        @material_switch

        gua_uint_gbuffer_varying_0.x = gua_material_id;
        gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_position_varying.xyz, 1.0);

        // done with the vertex
        EmitVertex();
      }
      EndPrimitive();
  }
}
