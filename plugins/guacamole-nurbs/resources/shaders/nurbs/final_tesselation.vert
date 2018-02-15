@include "resources/shaders/common/header.glsl"
                                                   
///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////             
layout (location = 0) in vec3  position;   
layout (location = 1) in uint  index;      
layout (location = 2) in vec2  tesscoord;  
layout (location = 3) in vec3  final_tesselation;  
                                                   
///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
flat out vec3  vertex_position;                  
flat out uint  vertex_index;                     
flat out vec2  vertex_tessCoord; 
flat out vec3 vertex_final_tesselation;                     
    
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void main()                       
{                                 
    vertex_position  = position;        
    vertex_index     = index;           
    vertex_tessCoord = tesscoord;   
    vertex_final_tesselation = final_tesselation;  
}     