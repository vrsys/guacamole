@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

                                      
        // input attributes                          
        layout (location = 0) in vec3  in_position;  
        layout (location = 1) in uint  in_r;         
        layout (location = 2) in uint  in_g;         
        layout (location = 3) in uint  in_b;         
        layout (location = 4) in uint empty;         
        layout (location = 5) in float in_radius;    
        layout (location = 6) in vec3 in_normal;     


        uniform uint gua_material_id;                                                     
      //  uniform mat4 gua_projection_matrix;          
      //  uniform mat4 gua_view_matrix;                
      //  uniform mat4 gua_model_matrix;
      //  uniform mat4 gua_normal_matrix;               
                                                     
        out vec3 gua_point_color;                        
                                                     
                                                     
        void main()                                  
        {                   
                      
          gl_Position = gua_projection_matrix *      
                        gua_view_matrix *            
                        gua_model_matrix *           
                        vec4(in_position,1.0);       
                     
          

          //gl_PointSize = 10000.0f;
          //gl_Position = vec4(0.5,0.5,0.0,1.0);
                                
          gua_point_color = vec3((in_r)/255.0f,     
                             (in_g)/255.0f,     
                             (in_b)/255.0f);    
        }                                          
