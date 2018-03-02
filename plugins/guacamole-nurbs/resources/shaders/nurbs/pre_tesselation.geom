@include "resources/shaders/common/header.glsl"           


///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////                                                              
layout(triangles) in;                                 

in vec3  eval_position[3];                          
in uint  eval_index[3];                             
in vec2  eval_tesscoord[3];   
in vec3  eval_final_tesselation[3];          
                                                                                            
///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
layout(xfb_buffer = 0, points, max_vertices = 4) out;      

layout (xfb_offset=0)  out vec3 transform_position;    
layout (xfb_offset=12) out uint transform_index;       
layout (xfb_offset=16) out vec2 transform_tesscoord;                                                 
layout (xfb_offset=24) out vec3 transform_final_tesselation;     

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////
uniform samplerBuffer parameter_texture;       
uniform samplerBuffer attribute_texture;          

@include "resources/glsl/trimmed_surface/parametrization_uniforms.glsl"
@include "resources/glsl/common/obb_area.glsl"
@include "resources/shaders/nurbs/patch_attribute_ssbo.glsl"

///////////////////////////////////////////////////////////////////////////////
// methods
///////////////////////////////////////////////////////////////////////////////
@include "resources/glsl/math/horner_surface.glsl.frag"


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()                                                                          
{                                                                             
    vec2 maxmax_tesscoord = max(max(eval_tesscoord[0], eval_tesscoord[1]), eval_tesscoord[2]);
    vec2 minmin_tesscoord = min(min(eval_tesscoord[0], eval_tesscoord[1]), eval_tesscoord[2]);
                                                                                             
    vec2 minmax_tesscoord = vec2(minmin_tesscoord.x, maxmax_tesscoord.y);            
    vec2 maxmin_tesscoord = vec2(maxmax_tesscoord.x, minmin_tesscoord.y);            
                                                                                             
    vec2 tesscoords[4];                                                              
    tesscoords[0] = minmin_tesscoord;                                                
    tesscoords[1] = maxmin_tesscoord;                                                
    tesscoords[2] = maxmax_tesscoord;                                                
    tesscoords[3] = minmax_tesscoord;                                                
                                                                                             
    int i, index;                                                                    
    ivec4 order = ivec4(-1);                                                         
                                                                                             
    for ( i = 0; i <= 2; i++ )                                                       
    {                                                                                
        bool minx = eval_tesscoord[i].x == minmin_tesscoord.x;                          
        bool maxx = eval_tesscoord[i].x == maxmax_tesscoord.x;                          
        bool miny = eval_tesscoord[i].y == minmin_tesscoord.y;                          
        bool maxy = eval_tesscoord[i].y == maxmax_tesscoord.y;                          
                                                                                             
        int index = 2 * int(maxy) + int((minx && maxy) || (maxx && miny));           
                                                                                             
        order[index] = i;                                                            
    }                                                                                
                                                                                             
    // discard triangles                                                             
    if ( order[3] == -1 || order[2] == -1 ) {                                        
        return;                                                                      
    }                                                                                
                                                                                             
    vec2 new_tesscoord = (order[0] == -1) ?  minmin_tesscoord : maxmin_tesscoord;    
                                                                                             
    vec4 new_puv;                                                                    
    vec4 new_du, new_dv;                                                             
                       
    int surface_index   = 0;
    int surface_order_u = 0;
    int surface_order_v = 0;
    retrieve_patch_data(int(eval_index[0]), surface_index, surface_order_u, surface_order_v);
                                                                                           
    evaluateSurface ( parameter_texture,                                             
                      int(surface_index),                                            
                      int(surface_order_u),                                          
                      int(surface_order_v),                                          
                      new_tesscoord,                                                 
                      new_puv );                                                     
                                                                                                
    for ( int i = 0; i != 4; ++i )                                                   
    {                                                                                
        index               = order[i];                                                    
        transform_position 	= order[i] == -1 ? new_puv.xyz : eval_position[index];            
        transform_index 	  = eval_index[0];                                                  
        transform_tesscoord = tesscoords[i];      
        transform_final_tesselation = eval_final_tesselation[0];                        
        EmitVertex();                                                                
    }                                                                                
    EndPrimitive(); 
  }          
