#version 420 core                                  
#extension GL_NV_bindless_texture : require        
#extension GL_NV_gpu_shader5      : enable         
   
///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////                                                               
layout(triangles) in;                              
layout(triangle_strip, max_vertices = 3) out;                    
                                                           
flat in vec3 teBitangent[3];                       
flat in vec3 teTangent[3];                         
flat in uint teIndex[3];                           
flat in vec2 teTessCoord[3];                       
flat in vec4 teNormal[3];                          
flat in vec4 tePosition[3];                        
      
///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////                                                               
flat out uint gIndex;                              
out vec2      gTessCoord;                          
out vec3      gua_position_varying;          

// built-in output
out vec2      gua_texcoords;        
                                        
out vec3      gua_world_normal;     
out vec3      gua_world_position;   
out vec3      gua_world_tangent;    
out vec3      gua_world_bitangent;  
                                        
out vec3      gua_object_normal;    
out vec3      gua_object_position;  
out vec3      gua_object_tangent;   
out vec3      gua_object_bitangent; 

// generic output
@output_definition
                                                                                               
///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////                                       
layout (std140, binding=0) uniform cameraBlock     
{                                                  
  mat4 gua_view_matrix;                            
  mat4 gua_projection_matrix;                      
  mat4 gua_inverse_projection_matrix;              
  mat4 gua_inverse_projection_view_matrix;         
  vec3 gua_camera_position;                        
};              
                                   
uniform mat4 gua_model_matrix;                     
uniform mat4 gua_normal_matrix;                    
uniform uint gua_material_id;                                                       
                                                           
uniform samplerBuffer parameter_texture;           
uniform samplerBuffer attribute_texture;        

// placeholder for generic uniforms
@uniform_definition

///////////////////////////////////////////////////////////////////////////////
// functions
///////////////////////////////////////////////////////////////////////////////
#include "shaders/uber_shaders/common/get_sampler_casts.glsl"

uint gua_get_material_id() {
  return gua_material_id;
}

// placeholder for generic user-defined functions
@material_methods


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()
{
  for ( int i = 0; i != 3; ++i )
  {
    gIndex      = teIndex[i];
    gTessCoord  = teTessCoord[i];

    // write built-in input for material
    ///////////////////////////////////////////////////////
    gua_texcoords  = gTessCoord;

    gua_position_varying = (gua_model_matrix * tePosition[i]).xyz;
    gua_object_normal    = teNormal[i].xyz;
    gua_object_tangent   = teTangent[i].xyz;
    gua_object_bitangent = teBitangent[i].xyz;
    gua_object_position  = tePosition[i].xyz;

    vec4 world_normal    = gua_normal_matrix * vec4 (teNormal[i].xyz, 0.0);
    gua_world_normal     = normalize ( world_normal.xyz );
    gua_world_tangent    = normalize ( gua_normal_matrix * vec4 (teTangent[i].xyz, 0.0) ).xyz;
    gua_world_bitangent  = normalize ( gua_normal_matrix * vec4 (teBitangent[i].xyz, 0.0) ).xyz;
    gua_world_position   = (gua_model_matrix * tePosition[i]).xyz;
    ///////////////////////////////////////////////////////
                      
    gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_position_varying.xyz, 1.0);

    // generic per-vertex material switch
    @material_switch

    gua_uint_gbuffer_varying_0.x = gua_material_id;    
     
    EmitVertex();                                                                               
  }                                                                                               
  EndPrimitive();                                                                                         
}                     