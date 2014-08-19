/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/renderer/NURBSUberShader.hpp>

// guacamole headers
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/renderer/NURBSShader.hpp>
#include <gua/renderer/GuaMethodsFactory.hpp>
#include <gua/renderer/NURBSRessource.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <sstream>
#include <list>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void NURBSUberShader::create(std::set<std::string> const& material_names)
{
  UberShader::create(material_names);

  // create shader for predraw pass to pre-tesselate if necessary
  std::vector<ShaderProgramStage>   pre_shader_stages;
  std::list<std::string>            interleaved_stream_capture;

  pre_shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, _transform_feedback_vertex_shader()));
  pre_shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_TESS_CONTROL_SHADER, _transform_feedback_tess_control_shader()));
  pre_shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_TESS_EVALUATION_SHADER, _transform_feedback_tess_evaluation_shader()));
  pre_shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, _transform_feedback_geometry_shader()));

  interleaved_stream_capture.push_back("xfb_position");
  interleaved_stream_capture.push_back("xfb_index");
  interleaved_stream_capture.push_back("xfb_tesscoord");

  auto xfb_program = std::make_shared<ShaderProgram>();
  xfb_program->set_shaders(pre_shader_stages, interleaved_stream_capture, true);
  add_program(xfb_program);

  // create final ubershader that writes to gbuffer
  std::vector<ShaderProgramStage> shader_stages;
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          _final_vertex_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_TESS_CONTROL_SHADER,    _final_tess_control_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_TESS_EVALUATION_SHADER, _final_tess_evaluation_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_GEOMETRY_SHADER,        _final_geometry_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        _final_fragment_shader()));

  auto final_program = std::make_shared<ShaderProgram>();
  final_program->set_shaders(shader_stages);
  add_program(final_program);
}


////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_transform_feedback_vertex_shader () const
{
    std::stringstream tf_vertex;

    tf_vertex << R"(                       
        #version 420 core                          
        #extension GL_NV_gpu_shader5 : enable 
                                                   
        // input attributes                        
        layout (location = 0) in vec3  position;   
        layout (location = 1) in uint  index;      
        layout (location = 2) in vec4  tesscoord;  
                                                   
        // output attributes                       
        flat out vec3  vPosition;                  
        flat out uint  vIndex;                    
        flat out vec2  vTessCoord;                 
                                                   
        void main()                                
        {                                          
          vPosition  = position;                   
          vIndex     = index;                      
          vTessCoord = tesscoord.xy;               
        }                                          
    )";

    return tf_vertex.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_transform_feedback_tess_control_shader () const
{
    std::stringstream tess_control;
    tess_control << R"(
                                                            
        #version 420 core                                   
        #extension GL_NV_gpu_shader5      : enable          
                                                            
        #define ID gl_InvocationID                          
                                                            
        layout(vertices = 4) out;                           
                                                            
        uniform samplerBuffer parameter_texture;            
        uniform samplerBuffer attribute_texture;            
                                                            
        // uniforms                                         
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
                                                            
        uniform float gua_texel_width;                      
        uniform float gua_texel_height;                     
        uniform float gua_tesselation_max_error;            
                                                            
        flat in vec3  vPosition[];                          
        flat in uint  vIndex[];                             
        flat in vec2  vTessCoord[];                         
                                                            
        flat out vec3 tcPosition[];                         
        flat out uint tcIndex[];                            
        flat out vec2 tcTessCoord[];                                                                             
    )";

    tess_control << NURBSShader::edge_length();
    tess_control << NURBSShader::control_polygon_length();
    tess_control << NURBSShader::edge_tess_level();
    tess_control << NURBSShader::is_inside();
    tess_control << NURBSShader::frustum_cull();

    tess_control << R"(
                                                                                         
        void main()                                                                      
        {                                                                                
          tcPosition[ID]  = vPosition[ID];                                               
          tcIndex[ID]     = vIndex[ID];                                                  
          tcTessCoord[ID] = vTessCoord[ID];                                              
                                                                                         
          mat4 mvp_matrix = gua_projection_matrix * gua_view_matrix * gua_model_matrix;  
                                                                                         
          // if ( frustum_cull ( mvp_matrix ) ) {                                        
          if ( true )                                                                    
          {                                                                              
            const float max_tesselation_per_pass = 64.0f;                                
                                                                                         
            vec4 data = texelFetch(attribute_texture, int(vIndex[ID]) * 5);              
                                                                                         
            uint surface_index   = floatBitsToUint(data.x);                              
            uint surface_order_u = floatBitsToUint(data.y);                              
            uint surface_order_v = floatBitsToUint(data.z);                              
                                                                                         
            vec4 edgelen = control_polygon_length(parameter_texture,                     
                                                  mvp_matrix,                            
                                                  int(surface_index),                    
                                                  int(surface_order_u),                  
                                                  int(surface_order_v),                  
                                                  int(1.0f/gua_texel_width),             
                                                  int(1.0f/gua_texel_height));           
                                                                                         
            //      3                                                                    
            //  3------2                                                                 
            //  |      |                                                                 
            //0 |      | 2                                                               
            //  |      |                                                                 
            //  0------1                                                                 
            //      1                                                                    
                                                                                         
            const float max_error_pre_tesselation = max_tesselation_per_pass *           
                                                    gua_tesselation_max_error;           
                                                                                         
            float edge0 = edge_tesslevel(edgelen[0], max_error_pre_tesselation );        
            float edge1 = edge_tesslevel(edgelen[1], max_error_pre_tesselation );        
            float edge2 = edge_tesslevel(edgelen[2], max_error_pre_tesselation );        
            float edge3 = edge_tesslevel(edgelen[3], max_error_pre_tesselation );        
                                                                                         
            float pre_tess_level = max(max(edge0, edge1), max(edge2, edge3));            
                                                                                         
            //Following three must be same for Ist Pass                                  
            gl_TessLevelInner[0] = pre_tess_level;                                       
            gl_TessLevelOuter[1] = pre_tess_level;                                       
            gl_TessLevelOuter[3] = pre_tess_level;                                       
                                                                                         
            //Following three must be same for Ist Pass                                  
            gl_TessLevelInner[1] = pre_tess_level;                                       
            gl_TessLevelOuter[0] = pre_tess_level;                                       
            gl_TessLevelOuter[2] = pre_tess_level;                                       
                                                                                         
          } else {                                                                       
            // constant tesselation -> debugging only                                    
            const float fixed_pre_tesselation = 2.0f;                                    
                                                                                         
            gl_TessLevelInner[0] = fixed_pre_tesselation;                                
            gl_TessLevelInner[1] = fixed_pre_tesselation;                                
                                                                                         
            gl_TessLevelOuter[0] = fixed_pre_tesselation;                                
            gl_TessLevelOuter[1] = fixed_pre_tesselation;                                
            gl_TessLevelOuter[2] = fixed_pre_tesselation;                                
            gl_TessLevelOuter[3] = fixed_pre_tesselation;                                
          }                                                                              
        }                                                                                
    )";

    return tess_control.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_transform_feedback_tess_evaluation_shader () const
{
    std::stringstream tess_eval;

    tess_eval << R"(
        #version 420 core                                   
        #extension GL_NV_gpu_shader5      : enable          
                                                            
        layout(quads, equal_spacing, ccw) in;               
                                                            
        flat in vec3  tcPosition[];                         
        flat in uint  tcIndex[];                            
        flat in vec2  tcTessCoord[];                        
                                                            
        flat out vec3 tePosition;                           
        flat out uint teIndex;                              
        flat out vec2 teTessCoord;                          
                                                            
        // uniforms                                         
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
                                                            
        uniform samplerBuffer parameter_texture;            
        uniform samplerBuffer attribute_texture;            
    )";

    tess_eval << NURBSShader::surface_horner_evaluation();

    tess_eval << R"(
                                                                               
        void main()                                                            
        {                                                                      
          vec2 p1 = mix(tcTessCoord[0].xy, tcTessCoord[1].xy, gl_TessCoord.x); 
          vec2 p2 = mix(tcTessCoord[3].xy, tcTessCoord[2].xy, gl_TessCoord.x); 
          vec2 uv = clamp(mix(p1, p2, gl_TessCoord.y), 0.0, 1.0);              
                                                                               
          vec4 data = texelFetch(attribute_texture, int(tcIndex[0]) * 5);      
          uint surface_index   = floatBitsToUint(data.x);                      
          uint surface_order_u = floatBitsToUint(data.y);                      
          uint surface_order_v = floatBitsToUint(data.z);                      
                                                                               
          vec4 puv, du, dv;                                                    
          evaluateSurface(parameter_texture,                                   
                          int(surface_index),                                  
                          int(surface_order_u),                                
                          int(surface_order_v),                                
                          uv,                                                  
                          puv);                                                
                                                                               
          tePosition  = puv.xyz;                                               
          teIndex     = tcIndex[0];                                            
          teTessCoord = uv;                                                    
        }                                                                      
    )";

    return tess_eval.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_transform_feedback_geometry_shader () const
{
   std::stringstream tf_geom;

   tf_geom << R"(
        #version 420 core                                     
        #extension GL_NV_gpu_shader5      : enable            
                                                              
        layout(triangles) in;                                 
        layout(points, max_vertices = 4) out;                 
                                                              
        // in attributes                                      
        flat in vec3  tePosition[3];                          
        flat in uint  teIndex[3];                             
        flat in vec2  teTessCoord[3];                         
                                                              
        // out per fragment                                   
        layout (location = 0)       out vec3 xfb_position;    
        layout (location = 1) flat  out uint xfb_index;       
        layout (location = 2)       out vec2 xfb_tesscoord;   
                                                              
        uniform samplerBuffer parameter_texture;              
        uniform samplerBuffer attribute_texture;              
    )";

    tf_geom << NURBSShader::surface_horner_evaluation();

    tf_geom << R"(
        void main()                                                                          
        {                                                                                    
            vec2 maxmax_tesscoord = max(max(teTessCoord[0], teTessCoord[1]), teTessCoord[2]);
            vec2 minmin_tesscoord = min(min(teTessCoord[0], teTessCoord[1]), teTessCoord[2]);
                                                                                             
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
                bool minx = teTessCoord[i].x == minmin_tesscoord.x;                          
                bool maxx = teTessCoord[i].x == maxmax_tesscoord.x;                          
                bool miny = teTessCoord[i].y == minmin_tesscoord.y;                          
                bool maxy = teTessCoord[i].y == maxmax_tesscoord.y;                          
                                                                                             
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
                                                                                             
            vec4 data = texelFetch(attribute_texture, int(teIndex[0]) * 5);                  
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
                xfb_position 	= order[i] == -1 ? new_puv.xyz : tePosition[index];            
                xfb_index 	  = teIndex[0];                                                  
                xfb_tesscoord = tesscoords[i];                                               
                EmitVertex();                                                                
            }                                                                                
            EndPrimitive(); 
          }                                                                
    )";

    return tf_geom.str();
}


////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_vertex_shader () const
{
    std::stringstream vertex_shader;

    vertex_shader << R"(
        #version 420 core                          
        #extension GL_NV_gpu_shader5      : enable 
                                                   
        // hard-coded in attributes                
        layout (location = 0) in vec3  position;   
        layout (location = 1) in uint  index;      
        layout (location = 2) in vec2  tesscoord;  
                                                   
        // hard-coded output                       
        flat out vec3  vPosition;                  
        flat out uint  vIndex;                     
        flat out vec2  vTessCoord;                 
    
        void main()                       
        {                                 
            vPosition  = position;        
            vIndex     = index;           
            vTessCoord = tesscoord;       
        }                                 
    )";

    return vertex_shader.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_tess_control_shader () const
{
    std::stringstream tess_ctrl;

    tess_ctrl << R"(
        #version 420 core                                 
        #extension GL_NV_gpu_shader5      : enable        
                                                          
        #define ID gl_InvocationID                        
                                                          
        layout(vertices = 4) out;                         
                                                          
        uniform samplerBuffer parameter_texture;          
        uniform samplerBuffer attribute_texture;          
        // uniforms                                       
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
        uniform float gua_tesselation_max_error;          
                                                          
        uniform float gua_texel_width;                    
        uniform float gua_texel_height;                   
                                                          
        flat in vec3  vPosition[];                        
        flat in uint  vIndex[];                           
        flat in vec2  vTessCoord[];                       
                                                          
        flat out uint tcIndex[];                          
        flat out vec2 tcTessCoord[];                      
    )";

    tess_ctrl << NURBSShader::to_screen_space();
    tess_ctrl << NURBSShader::edge_length();
    tess_ctrl << NURBSShader::control_polygon_length();
    tess_ctrl << NURBSShader::surface_horner_evaluation();
    tess_ctrl << NURBSShader::edge_tess_level();
    tess_ctrl << NURBSShader::inner_tess_level();
    tess_ctrl << NURBSShader::is_inside();
    tess_ctrl << NURBSShader::frustum_cull();

    tess_ctrl << R"(                                                                                                    
                                                                                                                                  
        void main()                                                                                                               
        {                                                                                                                         
          tcIndex[ID]     = vIndex[ID];                                                                                           
          tcTessCoord[ID] = vTessCoord[ID];                                                                                       
                                                                                                                                  
          mat4 mvp_matrix = gua_projection_matrix * gua_view_matrix * gua_model_matrix;                                           
                                                                                                                                  
          vec4 data = texelFetch(attribute_texture, int(vIndex[ID]) * 5);                                                         
          uint surface_index   = floatBitsToUint(data.x);                                                                         
          uint surface_order_u = floatBitsToUint(data.y);                                                                         
          uint surface_order_v = floatBitsToUint(data.z);                                                                         
                      
	  if (true)
	  {
            gl_TessLevelInner[0] = 8.0f;                                        
            gl_TessLevelOuter[1] = 8.0f;                                                                                         
            gl_TessLevelOuter[3] = 8.0f;                                                                                         
                                   
            gl_TessLevelInner[1] = 8.0f;                                            
            gl_TessLevelOuter[0] = 8.0f;                                                                                         
            gl_TessLevelOuter[2] = 8.0f;      
            return;  
          }
                                                                                                            
          //if ( abs(vTessCoord[0].x - vTessCoord[1].x) * abs(vTessCoord[1].y - vTessCoord[2].y) == 1.0 )                         
          if ( true )                                                                                                             
          {                                                                                                                       
            vec4 curve_factor = clamp(texelFetch(attribute_texture, int(vIndex[ID]) * 5 + 4), 1, 4);                              
            vec4 edgelen = control_polygon_length ( parameter_texture,                                                            
                                                    mvp_matrix,                                                                   
                                                    int(surface_index),                                                           
                                                    int(surface_order_u),                                                         
                                                    int(surface_order_v),                                                         
                                                    int(1.0f/gua_texel_width),                                                    
                                                    int(1.0f/gua_texel_height) );                                                 
                                                                                                                                  
            //     3                                                                                                             
            //  3------2                                                                                                         
            //  |      |                                                                                                          
            // 0|      |2                                                                                                        
            //  |      |                                                                                                          
            //  0------1                                                                                                          
            //     1                                                                                                              
                                                                                                                                
            float edge0 = edge_tesslevel(edgelen[0], gua_tesselation_max_error);                                                  
            float edge1 = edge_tesslevel(edgelen[1], gua_tesselation_max_error);                                                  
            float edge2 = edge_tesslevel(edgelen[2], gua_tesselation_max_error);                                                 
            float edge3 = edge_tesslevel(edgelen[3], gua_tesselation_max_error);                                                  
                                                                                                                               
            //Following three must be same for Ist Pass                                                                         
            gl_TessLevelInner[0] = inner_tess_level(attribute_texture, int(vIndex[ID]) * 5,                                       
                                                    mvp_matrix,                                                                   
                                                    gua_tesselation_max_error,                                                    
                                                    int(1.0f/gua_texel_width),                                                    
                                                    int(1.0f/gua_texel_height));                                                  
            gl_TessLevelOuter[1] = edge1;                                                                                         
            gl_TessLevelOuter[3] = edge3;                                                                                         
                                                                                                                                  
            //Following three must be same for Ist Pass                                                                           
            gl_TessLevelInner[1] = inner_tess_level(attribute_texture, int(vIndex[ID]) * 5,                                       
                                                    mvp_matrix,                                                                   
                                                    gua_tesselation_max_error,                                                    
                                                    int(1.0f/gua_texel_width),                                                    
                                                    int(1.0f/gua_texel_height));                                                  
            gl_TessLevelOuter[0] = edge0;                                                                                         
            gl_TessLevelOuter[2] = edge2;    
        )";

        tess_ctrl << R"(                
          } else {                                                                                                                
                                                                                                                                  
            int width  = int(1.0f/gua_texel_width);                                                                               
            int height = int(1.0f/gua_texel_height);                                                                              
                                                                                                                                  
            vec4 point_on_plane0 = to_screen_space(vPosition[0], mvp_matrix, width, height);                                      
            vec4 point_on_plane1 = to_screen_space(vPosition[1], mvp_matrix, width, height);                                      
            vec4 point_on_plane2 = to_screen_space(vPosition[2], mvp_matrix, width, height);                                      
            vec4 point_on_plane3 = to_screen_space(vPosition[3], mvp_matrix, width, height);                                      
                                                                                                                                  
            // Approach I ->                                                                                                      
            // For Outer Tessellation Levels : Take ratio according to the original control polygon length.                       
            // For Inner Tessellation Levels : Evaluate the mid point of the patch and get the diagonal length.                   
                                                                                                                                  
            vec4 edgelen = control_polygon_length(parameter_texture,                                                              
                                                  mvp_matrix,                                                                     
                                                  int(surface_index),                                                             
                                                  int(surface_order_u),                                                           
                                                  int(surface_order_v),                                                           
                                                  width,                                                                          
                                                  height);                                                                        
                                                                                                                                  
            vec2 p1 = mix(vTessCoord[0].xy, vTessCoord[1].xy, 0.5);                                                               
            vec2 p2 = mix(vTessCoord[3].xy, vTessCoord[2].xy, 0.5);                                                               
                                                                                                                                  
            vec2 mid_uv = mix(p1, p2, 0.5);                                                                                       
            vec4 mid_p;                                                                                                           
                                                                                                                                  
            evaluateSurface(parameter_texture,                                                                                    
                            int(surface_index),                                                                                   
                            int(surface_order_u),                                                                                 
                            int(surface_order_v),                                                                                 
                            mid_uv,                                                                                               
                            mid_p);                                                                                               
                                                                                                                                  
            vec4 mid_p_screenspace = to_screen_space(mid_p.xyz, mvp_matrix, width, height);                                       
                                                                                                                                  
            float length1 = length(point_on_plane0.xy - mid_p_screenspace.xy) +                                                   
                            length(mid_p_screenspace.xy - point_on_plane3.xy);                                                    
                                                                                                                                  
            float length2 = length(point_on_plane2.xy - mid_p_screenspace.xy) +                                                   
                            length(mid_p_screenspace.xy - point_on_plane1.xy);                                                    
                                                                                                                                  
            float diagonal_length = min(length1, length2);                                                                        
                                                                                                                                  
            float edge01 = edge_tesslevel(mix(edgelen[0], edgelen[2], abs(vTessCoord[0].y - vTessCoord[2].y)), gua_tesselation_max_error); 
            float edge32 = edge_tesslevel(mix(edgelen[0], edgelen[2], abs(vTessCoord[0].y - vTessCoord[2].y)), gua_tesselation_max_error); 
            float edge13 = edge_tesslevel(mix(edgelen[1], edgelen[3], abs(vTessCoord[0].x - vTessCoord[1].x)), gua_tesselation_max_error); 
            float edge20 = edge_tesslevel(mix(edgelen[1], edgelen[3], abs(vTessCoord[0].x - vTessCoord[1].x)), gua_tesselation_max_error); 
                                                                                                                                  
            // Approach II ->                                                                                                     
            // For Outer Tessellation Levels : Approx. the curvature length of the edge according to the angle betw. its normals.
            // For Inner Tessellation Levels : Approx. the curvature of the surface according to the all edge normals.            
                                                                                                                                  
            //Following three must be same for Ist Pass                                                                           
            gl_TessLevelInner[0] = edge_tesslevel(diagonal_length, gua_tesselation_max_error);                                             
            gl_TessLevelOuter[1] = edge01;                                                                                        
            gl_TessLevelOuter[3] = edge32;                                                                                        
                                                                                                                                  
            //Following three must be same for Ist Pass                                                                           
            gl_TessLevelInner[1] = edge_tesslevel(diagonal_length, gua_tesselation_max_error);                                             
            gl_TessLevelOuter[0] = edge20;                                                                                        
            gl_TessLevelOuter[2] = edge13;                                                                                        
          }                                                                                                                       
        }                                                                                                                        
    )";

    return tess_ctrl.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_tess_evaluation_shader () const
{
    std::stringstream tess_eval;

    tess_eval << R"(
        #version 420 core                                   
        #extension GL_NV_gpu_shader5      : enable          
                                                            
        layout(quads, equal_spacing, ccw) in;               
                                                            
        flat in uint  tcIndex[];                            
        flat in vec2  tcTessCoord[];                        
                                                            
        flat out vec3   teBitangent;                        
        flat out vec3   teTangent;                          
        flat out uint   teIndex;                            
        flat out vec2   teTessCoord;                        
        flat out vec4   teNormal;                           
        flat out vec4   tePosition;                         
                                                            
        // uniforms                                         
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
                                                            
        uniform samplerBuffer parameter_texture;            
        uniform samplerBuffer attribute_texture;            
    )";

    tess_eval << NURBSShader::surface_horner_evaluation();

    tess_eval << R"(
                                                                               
        void main()                                                            
        {                                                                      
          vec4 p, du, dv;                                                      
                                                                               
          vec4 data = texelFetch(attribute_texture, int(tcIndex[0]) * 5);      
          uint surface_index   = floatBitsToUint(data.x);                      
          uint surface_order_u = floatBitsToUint(data.y);                      
          uint surface_order_v = floatBitsToUint(data.z);                      
                                                                               
          vec2 p1 = mix(tcTessCoord[0].xy, tcTessCoord[1].xy, gl_TessCoord.x); 
          vec2 p2 = mix(tcTessCoord[3].xy, tcTessCoord[2].xy, gl_TessCoord.x); 
                                                                               
          vec2 uv;                                                             
                                                                               
          uv = clamp(mix(p1, p2, gl_TessCoord.y), 0.0, 1.0);                   
                                                                               
          evaluateSurface(parameter_texture,                                   
                          int(surface_index),                                  
                          int(surface_order_u),                                
                          int(surface_order_v),                                
                          uv, p, du, dv);                                      
                                                                               
          tePosition  = vec4(p.xyz, 1.0);                                      
          teBitangent = normalize(du.xyz);                                     
          teTangent   = normalize(dv.xyz);                                     
          teIndex     = tcIndex[0];                                            
          teTessCoord = uv;                                                    
          teNormal    = vec4(normalize(cross(du.xyz, dv.xyz)), 0.0);           
                                                                               
          vec4 nview  = gua_normal_matrix * teNormal;                          
          vec4 pview  = gua_view_matrix * gua_model_matrix * tePosition;       
                                                                               
          if ( dot(normalize(nview.xyz), -normalize(pview.xyz)) < 0.0f ) {     
            teNormal = -teNormal;                                              
          }                                                                    
        }                                                                      
    )";

    return tess_eval.str();
}


////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_geometry_shader () const
{
    std::stringstream geom_shader;

    geom_shader << R"(
        #version 420 core                                  
        #extension GL_NV_bindless_texture : require        
        #extension GL_NV_gpu_shader5      : enable         
                                                           
        layout(triangles) in;                              
        layout(triangle_strip, max_vertices = 3) out;      
                                                           
        // uniforms                                        
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
                                                           
        flat in vec3 teBitangent[3];                       
        flat in vec3 teTangent[3];                         
        flat in uint teIndex[3];                           
        flat in vec2 teTessCoord[3];                       
        flat in vec4 teNormal[3];                          
        flat in vec4 tePosition[3];                        
                                                           
        flat out uint gIndex;                              
        out vec2      gTessCoord;                          
                                                           
        uniform samplerBuffer parameter_texture;           
        uniform samplerBuffer attribute_texture;           
    )";

    // generated material-dependent uniform definitions
    geom_shader << fshader_factory_->get_uniform_mapping().get_uniform_definition();
    geom_shader << std::endl;

    // generated varying variables
    geom_shader << "out vec3 gua_position_varying;" << std::endl;
    geom_shader << vshader_factory_->get_output_mapping().get_gbuffer_output_definition(false, true);
    geom_shader << std::endl;

    // hard-coded buit-in output variables -> should be written!
    geom_shader << R"(
        vec2 gua_texcoords;        
                                   
        vec3 gua_world_normal;     
        vec3 gua_world_position;   
        vec3 gua_world_tangent;    
        vec3 gua_world_bitangent;  
                                   
        vec3 gua_object_normal;    
        vec3 gua_object_position;  
        vec3 gua_object_tangent;   
        vec3 gua_object_bitangent; 
    )";

    // generated auxiliary methods
    GuaMethodsFactory method_factory;
    geom_shader << method_factory.get_sampler_casts() << std::endl;

    // material specific methods -----------------------------------------------
    for (auto const& method : vshader_factory_->get_main_functions()) {
        geom_shader << method.second << std::endl;
    }

    geom_shader << R"(
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
    )";

    // generated code
    auto main_calls(vshader_factory_->get_main_calls());
    unsigned current_case(0);
    unsigned cases_per_block(CASES_PER_UBERSHADER_SWITCH);

    auto call(main_calls.begin());

    while (current_case < main_calls.size()) {

        geom_shader << "switch(gua_material_id) {" << std::endl;

        for (unsigned i(current_case); i < main_calls.size() && i < current_case+cases_per_block; ++i) {
            geom_shader << " case " << call->first << ": " << call->second << " break;" << std::endl;
            ++call;
        }

        geom_shader << "}" << std::endl;


        current_case += cases_per_block;
    }

    geom_shader << R"(
                        gua_uint_gbuffer_varying_0.x = gua_material_id;                                             
                        gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_position_varying.xyz, 1.0);
                        EmitVertex();                                                                               
                    }                                                                                               
            EndPrimitive();                                                                                         
        }                                                                                                           
    )";

    return geom_shader.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_fragment_shader () const
{
    std::string fragment_shader;

    fragment_shader += R"(
        #version 420 core
        #extension GL_NV_bindless_texture : require
        #extension GL_NV_gpu_shader5 : enable

        #define TRIM_ERROR_TOLERANCE 0.00001

        precision highp float;

        flat in uint gIndex;
        in vec2      gTessCoord;

        uniform samplerBuffer attribute_texture;

        uniform samplerBuffer trim_partition;
        uniform samplerBuffer trim_contourlist;
        uniform samplerBuffer trim_curvelist;
        uniform samplerBuffer trim_curvedata;
        uniform samplerBuffer trim_pointdata;
    )";

    fragment_shader += NURBSShader::curve_horner_evaluation();
    fragment_shader += NURBSShader::binary_search();
    fragment_shader += NURBSShader::bisect_curve();
    fragment_shader += NURBSShader::trimming_helper_methods();
    fragment_shader += NURBSShader::bisect_contour();
    fragment_shader += NURBSShader::contour_binary_search();
    fragment_shader += NURBSShader::contour_based_trimming();

    // input from vertex shader ------------------------------------------
    fragment_shader += R"(

        in vec3 gua_position_varying;

        //***** generated input defintion
        @input_definition

        // uniforms
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

        uniform float gua_texel_width;
        uniform float gua_texel_height;


        //***** generated uniform definition
        @uniform_definition

        //***** generated output definition
        @output_definition

        // global gua_* methods
        vec2 gua_get_quad_coords() {
            return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
        }
    )";

    GuaMethodsFactory method_factory;
    fragment_shader += method_factory.get_sampler_casts();

    fragment_shader += R"(
        uint gua_get_material_id() {
            return gua_uint_gbuffer_varying_0.x;
        }

        vec3 gua_get_position() {
            return gua_position_varying;
        }

        //***** generated material methods
        @material_methods

        // main switch
        void main()
        {
            vec4 data = texelFetch(attribute_texture, int(gIndex) * 5);
            uint trim_index = floatBitsToUint(data.w);

            vec4 nurbs_domain = texelFetch(attribute_texture, int(gIndex) * 5 + 1);

            vec2 domain_size  = vec2(nurbs_domain.z - nurbs_domain.x, nurbs_domain.w - nurbs_domain.y);

            vec2 uv_nurbs     = gTessCoord.xy * domain_size + nurbs_domain.xy;

            int tmp = 0;
            bool trimmed      = trim (trim_partition,
                                      trim_contourlist,
                                      trim_curvelist,
                                      trim_curvedata,
                                      trim_pointdata,
                                      uv_nurbs,
                                      int(trim_index), 1, tmp, 0.0001f, 16);
            if ( trimmed ) {
                discard;
            }

          @material_switch

          gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;
        }
    )";

    // input from vertex shader
    string_utils::replace(fragment_shader, "@input_definition",
      vshader_factory_->get_output_mapping().get_gbuffer_output_definition(true, true));

    // material specific uniforms
    string_utils::replace(fragment_shader, "@uniform_definition",
      get_uniform_mapping()->get_uniform_definition());
    // BOOST_FOREACH (auto method, fragment_shader_factory_->get_main_functions()) {
    //     fragment_shader << method.second << std::endl;
    // }

    // outputs
    string_utils::replace(fragment_shader, "@output_definition",
      get_gbuffer_mapping()->get_gbuffer_output_definition(false, false));

    // print material specific methods
    string_utils::replace(fragment_shader, "@material_methods",
      UberShader::print_material_methods(*fshader_factory_));

    // print main switch(es)
    string_utils::replace(fragment_shader, "@material_switch",
      UberShader::print_material_switch(*fshader_factory_));

    return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ GeometryUberShader::stage_mask NURBSUberShader::get_stage_mask() const
{
  return GeometryUberShader::DRAW_STAGE;
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void NURBSUberShader::preframe(RenderContext const& ctx) const
{
  throw std::runtime_error("NURBSUberShader::preframe(): not implemented");
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void NURBSUberShader::predraw(RenderContext const& ctx,
                                          std::string const& ksfile_name,
                                          std::string const& material_name,
                                          scm::math::mat4 const& model_matrix,
                                          scm::math::mat4 const& normal_matrix,
                                          Frustum const& /*frustum*/,
                                          View const& view) const
{
  throw std::runtime_error("NURBSUberShader::predraw(): not implemented");
}


////////////////////////////////////////////////////////////////////////////////

void NURBSUberShader::draw(RenderContext const& ctx,
                           std::string const& filename,
                           std::string const& material_name,
                           scm::math::mat4 const& model_matrix,
                           scm::math::mat4 const& normal_matrix,
                           Frustum const& /*frustum*/,
                           View const& view) const
{

  auto geometry = std::static_pointer_cast<NURBSRessource>(GeometryDatabase::instance()->lookup(filename));
  auto material = MaterialDatabase::instance()->lookup(material_name);

  //set_uniform(ctx, 8, "gua_max_tesselation");
  set_uniform(ctx, material->get_id(), "gua_material_id");
  set_uniform(ctx, model_matrix, "gua_model_matrix");
  set_uniform(ctx, normal_matrix, "gua_normal_matrix");

  #ifdef DEBUG_XFB_OUTPUT
    scm::gl::transform_feedback_statistics_query_ptr q = ctx
      .render_device->create_transform_feedback_statistics_query(0);
    ctx.render_context->begin_query(q);
#endif

  // pre-tesselate if necessary
  get_program(transform_feedback_pass)->use(ctx);
  {
    ctx.render_context->apply();
    geometry->predraw(ctx);
  }
  get_program(transform_feedback_pass)->unuse(ctx);

#ifdef DEBUG_XFB_OUTPUT
    ctx.render_context->end_query(q);
    ctx.render_context->collect_query_results(q);
    std::cout << q->result()._primitives_generated << " , "
      << q->result()._primitives_written << std::endl;
#endif

  // invoke tesselation/trim shader for adaptive nurbs rendering
  get_program(final_pass)->use(ctx);
  {
    ctx.render_context->apply();
    geometry->draw(ctx);
  }
  get_program(final_pass)->unuse(ctx);

}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void NURBSUberShader::postdraw(RenderContext const& ctx,
  std::string const& ksfile_name,
  std::string const& material_name,
  scm::math::mat4 const& model_matrix,
  scm::math::mat4 const& normal_matrix,
  Frustum const& /*frustum*/,
  View const& view) const
{
  throw std::runtime_error("NURBSUberShader::postdraw(): not implemented");
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void NURBSUberShader::postframe(RenderContext const& ctx) const
{
  throw std::runtime_error("NURBSUberShader::postframe(): not implemented");
}


}
