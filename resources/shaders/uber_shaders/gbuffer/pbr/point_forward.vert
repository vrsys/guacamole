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

        uniform float height_divided_by_top_minus_bottom;
        uniform float near_plane;         
        
        //output to fragment shader                                             
        out vec3 gua_point_color;
        out vec3 gua_normal;                        
                                                     
                                                     
        void main()                                  
        {                   
                      
          vec4 pos_es = gua_view_matrix * gua_model_matrix * vec4(in_position, 1.0);

          gl_Position = gua_projection_matrix * pos_es;  
          

          gl_PointSize = 2.0f * in_radius * (near_plane/-pos_es.z)* height_divided_by_top_minus_bottom;

          
                                
          gua_point_color = vec3((in_r)/255.0f,     
                             (in_g)/255.0f,     
                             (in_b)/255.0f);

          gua_normal = (gua_normal_matrix * vec4(in_normal, 0.0)).xyz;    
        }                                          
