@include "resources/shaders/common/header.glsl" 
   
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

@include "resources/shaders/common/gua_global_variable_declaration.glsl"

///////////////////////////////////////////////////////////////////////////////
// guacamole vertex output interface
///////////////////////////////////////////////////////////////////////////////
// vec3  gua_world_position;
// vec3  gua_view_position;
// vec3  gua_normal;
// vec3  gua_tangent;
// vec3  gua_bitangent;
// vec2  gua_texcoords;
// vec3  gua_color;
// float gua_roughness;
// float gua_metalness;
// float gua_emissivity;
// bool  gua_flags_passthrough;
// float gua_alpha;
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// built-in output 
///////////////////////////////////////////////////////////////////////////////     
@include "resources/shaders/common/gua_vertex_shader_output.glsl"
                                                                          
///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////                                                                      
@include "resources/glsl/trimmed_surface/parametrization_uniforms.glsl"    
@include "resources/shaders/common/gua_camera_uniforms.glsl"

@material_uniforms@

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
@include "resources/shaders/nurbs/patch_attribute_ssbo.glsl"


@material_method_declarations_vert@

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()
{
  @material_input@

  // force vertex order to be face forward! 
  mat4 modelview = gua_view_matrix * gua_model_matrix;

  vec4 nurbs_domain = retrieve_patch_domain(int(teIndex[0]));
  vec2 domain_size  = vec2(nurbs_domain.z - nurbs_domain.x, nurbs_domain.w - nurbs_domain.y);

  // force vertex order to be face forward to prevent backface culling!!!
  vec3 a_view_space = (modelview * tePosition[0]).xyz;
  vec3 b_view_space = (modelview * tePosition[1]).xyz;
  vec3 c_view_space = (modelview * tePosition[2]).xyz;

  vec3 normal_view_space = cross(normalize(b_view_space - a_view_space), normalize(c_view_space - b_view_space));
  vec4 normal_world_space = gua_view_matrix * vec4(normal_view_space, 0.0);

  int vertex_id_first = 0;
  int vertex_id_last  = 3;
  int increment       = 1;

  bool invert_vertex_order = dot(normal_view_space, normalize(-a_view_space)) <= 0.0;
  float inversion_factor = invert_vertex_order ? -1.0 : 1.0;

  if (invert_vertex_order) {
    vertex_id_first = 2;
    vertex_id_last  = -1;
    increment       = -1;
  }

  for ( int i = vertex_id_first; i != vertex_id_last; i = i + increment )
  {
    gIndex      = teIndex[i];

    // write built-in input for material
    ///////////////////////////////////////////////////////
    gua_world_position   = (gua_model_matrix * tePosition[i]).xyz;
    gua_normal           = inversion_factor * teNormal[i].xyz;
    gua_texcoords        = teTessCoord[i];
    gua_tangent          = normalize ( gua_normal_matrix * vec4 (teTangent[i].xyz, 0.0) ).xyz;
    gua_bitangent        = normalize ( gua_normal_matrix * vec4 (teBitangent[i].xyz, 0.0) ).xyz;
    
    gua_metalness      = 0;
    gua_roughness        = 50;
    gua_emissivity       = 0;
    ///////////////////////////////////////////////////////
                      
    gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_world_position.xyz, 1.0);

    @material_method_calls_vert@

    @include "resources/shaders/common/gua_varyings_assignment.glsl"

    EmitVertex();                                                                               
  }                                                                                               
  EndPrimitive();                                                                                         
}                     
