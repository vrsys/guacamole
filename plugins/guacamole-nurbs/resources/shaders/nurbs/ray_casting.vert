@include "resources/shaders/common/header.glsl"
                                                   
///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////             
layout (location = 0) in vec3 vertex;
layout (location = 1) in vec4 texcoord;
layout (location = 2) in vec4 vattrib0;
layout (location = 3) in vec4 vattrib1;
                                                   
///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
out vec4 v_modelcoord;
out vec4 frag_texcoord;

flat out int  trim_index_db;
flat out int  trim_index_cmb;
flat out int  data_index;
flat out int  order_u;
flat out int  order_v;

flat out vec4 uvrange;
flat out int  trimtype;

// built-in out
out vec3      gua_position_varying;           
                                        
out vec3      gua_world_normal;     
out vec3      gua_world_position;   
out vec3      gua_world_tangent;    
out vec3      gua_world_bitangent;  
                                        
out vec3      gua_object_normal;    
out vec3      gua_object_position;  
out vec3      gua_object_tangent;   
out vec3      gua_object_bitangent; 

// generic output
@include "resources/shaders/common/gua_global_variable_declaration.glsl"
@include "resources/shaders/common/gua_vertex_shader_output.glsl"

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////
@include "resources/shaders/common/gua_camera_uniforms.glsl"   

@material_uniforms@

@material_method_declarations_vert@

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()                       
{                  
  @material_input@
               
  v_modelcoord   = vec4(vertex, 1.0);
  frag_texcoord  = texcoord;

  trim_index_db  = int(floatBitsToUint(vattrib0[0]));
  trim_index_cmb = int(floatBitsToUint(vattrib0[3]));
  data_index     = int(floatBitsToUint(vattrib0[1]));
  order_u        = int(texcoord[2]);
  order_v        = int(texcoord[3]);

  uvrange        = vattrib1;
  trimtype       = int(floatBitsToUint(vattrib0[2]));

  // transform convex hull in modelview to generate fragments
  gl_Position    = gua_projection_matrix * gua_view_matrix * gua_model_matrix * v_modelcoord; 
  gua_position_varying = (gua_model_matrix * v_modelcoord).xyz;

  @material_method_calls_vert@

  @include "resources/shaders/common/gua_varyings_assignment.glsl"
}     