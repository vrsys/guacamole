@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

                                      
        // input attributes                          
        layout (location = 0) in vec3  in_position;  
        layout (location = 1) in float  in_r;      
        layout (location = 2) in float  in_g;  
        layout (location = 3) in float  in_b;  
        layout (location = 4) in float  empty;             
        layout (location = 5) in float in_radius;    
        layout (location = 6) in vec3 in_normal;     


        uniform uint gua_material_id;                                                     

        uniform float height_divided_by_top_minus_bottom;
        uniform float near_plane;         
        
        uniform float radius_model_scaling;

        //output to fragment shader                                             
        out vec3 pass_normal;
        out float pass_mv_vert_depth;
        out float pass_scaled_radius;  
        out float pass_view_scaling;          
                                                     
                                                     
        void main()                                  
        {                   
            
          float scaled_radius = radius_model_scaling * in_radius;

          mat4 model_view_matrix = gua_view_matrix * gua_model_matrix;

          vec4 pos_es = model_view_matrix * vec4(in_position, 1.0);

          vec4 pos_es_ub = model_view_matrix * vec4(in_position + vec3(0.0,  0.5  * in_radius, 0.0), 1.0);
          vec4 pos_es_lb = model_view_matrix * vec4(in_position + vec3(0.0, -0.5  * in_radius, 0.0), 1.0);

          float view_scaling = length(vec3(pos_es_ub) - vec3(pos_es_lb));
          pass_view_scaling = view_scaling;

          gl_Position = gua_projection_matrix * pos_es;  
          
          gl_PointSize = 2.0f * scaled_radius * (near_plane/-pos_es.z)* height_divided_by_top_minus_bottom;


          //gl_PointSize = 1.0;

          //pass_normal = (gua_normal_matrix * vec4(in_normal, 0.0)).xyz;
          pass_normal = normalize(( gua_normal_matrix * vec4(in_normal, 0.0)).xyz);   

          pass_mv_vert_depth = pos_es.z;
          pass_scaled_radius = scaled_radius;


        }                                          
