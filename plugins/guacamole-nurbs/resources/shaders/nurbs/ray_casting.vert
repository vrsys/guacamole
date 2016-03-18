@include "resources/shaders/common/header.glsl"
                                                   
///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////             
layout (location = 0) in vec3 chull_vertex;
layout (location = 1) in vec4 chull_uv_and_order;
layout (location = 2) in vec4 vattrib0;
layout (location = 3) in vec4 vattrib1;
                                                   
///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
out vec4 v_modelcoord;
out vec2 initial_uv_guess;
out vec3 position_varying;

flat out int  trim_index;
flat out int  trim_type;
flat out int  trim_approach;
flat out int  data_index;

flat out int  order_u;
flat out int  order_v;

flat out vec4 uvrange;

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
               
  v_modelcoord     = vec4(chull_vertex, 1.0);
  initial_uv_guess = chull_uv_and_order.xy;

  trim_index       = int(floatBitsToUint(vattrib0[0]));
  trim_approach    = int(floatBitsToUint(vattrib0[3]));
  trim_type        = int(floatBitsToUint(vattrib0[2]));
  data_index       = int(floatBitsToUint(vattrib0[1]));

  order_u          = int(floatBitsToUint(chull_uv_and_order[2]));
  order_v          = int(floatBitsToUint(chull_uv_and_order[3]));
    
  uvrange          = vattrib1;

  // transform convex hull in modelview to generate fragments
  
  position_varying = (gua_model_matrix * v_modelcoord).xyz;

  gua_world_position = (gua_model_matrix * vec4(chull_vertex, 1.0)).xyz;
  gua_view_position  = (gua_model_view_matrix * vec4(chull_vertex, 1.0)).xyz;
  gua_texcoords      = chull_uv_and_order.xy;
  gua_metalness      = 0;
  gua_roughness      = 0;
  gua_emissivity     = 0;

  @material_method_calls_vert@

  @include "resources/shaders/common/gua_varyings_assignment.glsl"

  gl_Position    = gua_projection_matrix * gua_view_matrix * gua_model_matrix * v_modelcoord; 
}     