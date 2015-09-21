@include "resources/shaders/common/header.glsl"           
       
///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////                                                              
layout(triangles) in;                                 

in vec3  eval_position[3];                          
in uint  eval_index[3];                             
in vec2  eval_tesscoord[3];         
                                                                                            
///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
layout(points, max_vertices = 4) out;      

layout (location = 0)       out vec3 xfb_position;    
layout (location = 1) flat  out uint xfb_index;       
layout (location = 2)       out vec2 xfb_tesscoord;   
                                                              

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////
uniform samplerBuffer parameter_texture;              
uniform samplerBuffer attribute_texture;    

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
                                                                                             
    vec4 data = texelFetch(attribute_texture, int(eval_index[0]) * 5);                  
    uint surface_index   = floatBitsToUint(data.x);                                  
    uint surface_order_u = floatBitsToUint(data.y);                                  
    uint surface_order_v = floatBitsToUint(data.z);                                  
                                                                                             
    evaluateSurface ( parameter_texture,                                             
                      int(surface_index),                                            
                      int(surface_order_u),                                          
                      int(surface_order_v),                                          
                      new_tesscoord,                                                 
                      new_puv );                                                     
                                                                                             
    for ( int i = 0; i != 4; ++i )                                                   
    {                                                                                
        index         = order[i];                                                    
        xfb_position 	= order[i] == -1 ? new_puv.xyz : eval_position[index];            
        xfb_index 	  = eval_index[0];                                                  
        xfb_tesscoord = tesscoords[i];                                               
        EmitVertex();                                                                
    }                                                                                
    EndPrimitive(); 
  }          