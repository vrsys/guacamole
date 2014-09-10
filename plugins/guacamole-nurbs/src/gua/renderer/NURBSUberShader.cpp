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
#include <gua/renderer/GuaMethodsFactory.hpp>
#include <gua/renderer/NURBSRessource.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>
#include <gpucast/core/config.hpp>

// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>

namespace gua {

  namespace {

    ///////////////////////////////////////////////////////////////////////////
    std::string read_shader_file(std::string const& path, std::vector<std::string> const& root_dirs)
    {
      try {
        for (auto const& root : root_dirs)
        {
          std::string full_path(root + std::string("/") + path);
          std::ifstream ifstr(full_path.c_str(), std::ios::in);

          if (ifstr.good()) {
            return std::string(std::istreambuf_iterator<char>(ifstr), std::istreambuf_iterator<char>());
          }
          ifstr.close();
        }
        throw std::runtime_error("File not found.");
      }
      catch (...) {
        std::cerr << "Error reading file : " << path << std::endl;
        return "";
      }
    }

    ///////////////////////////////////////////////////////////////////////////
    void resolve_includes(std::string& shader_source, std::vector<std::string> const& root_dirs)
    {
      std::size_t search_pos(0);

      std::string search("#include");

      while (search_pos != std::string::npos) 
      {
        search_pos = shader_source.find(search, search_pos);

        if (search_pos != std::string::npos) {

          std::size_t start(shader_source.find('\"', search_pos) + 1);
          std::size_t end(shader_source.find('\"', start));

          std::string file(shader_source.substr(start, end - start));

          std::string include = read_shader_file(file, root_dirs);
          shader_source.replace(search_pos, end - search_pos + 2, include);

          // advance search pos
          search_pos = search_pos + include.length();
        }
      }
    }
  }


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

  auto geometry_shader_template = _final_geometry_shader();
  _insert_generic_per_vertex_code(geometry_shader_template);
  shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, geometry_shader_template));

  auto fragment_shader_template = _final_fragment_shader();
  _insert_generic_per_fragment_code(fragment_shader_template);
  shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, fragment_shader_template));

  auto final_program = std::make_shared<ShaderProgram>();
  final_program->set_shaders(shader_stages);
  add_program(final_program);
}


////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_transform_feedback_vertex_shader () const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string vertex_shadercode = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/pre_tesselation.vert", root_dirs);
  resolve_includes(vertex_shadercode, root_dirs);

  return vertex_shadercode;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_transform_feedback_tess_control_shader () const
{
#if 0
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
        uniform float max_pre_tesselation;         
                                                            
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
          if ( false )                                                                    
          {                                                                              
            const float max_tesselation_per_pass = 16.0f;                                
                                                                                         
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
                                                                                         
            gl_TessLevelInner[0] = max_pre_tesselation;                                
            gl_TessLevelInner[1] = max_pre_tesselation;                                
                                                                                         
            gl_TessLevelOuter[0] = max_pre_tesselation;                                
            gl_TessLevelOuter[1] = max_pre_tesselation;                                
            gl_TessLevelOuter[2] = max_pre_tesselation;                                
            gl_TessLevelOuter[3] = max_pre_tesselation;                                
          }                                                                              
        }                                                                                
    )";

    return tess_control.str();
#else
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string tess_control_shadercode = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/pre_tesselation.tctrl", root_dirs);
  resolve_includes(tess_control_shadercode, root_dirs);

  return tess_control_shadercode;
#endif

}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_transform_feedback_tess_evaluation_shader () const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string tess_eval_shadercode = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/pre_tesselation.teval", root_dirs);
  resolve_includes(tess_eval_shadercode, root_dirs);

  return tess_eval_shadercode;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_transform_feedback_geometry_shader () const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string geometry_shadercode = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/pre_tesselation.geom", root_dirs);
  resolve_includes(geometry_shadercode, root_dirs);

  return geometry_shadercode;
}


////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_vertex_shader () const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_vertex_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.vert", root_dirs);
  resolve_includes(final_vertex_shader_code, root_dirs);

  return final_vertex_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_tess_control_shader () const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_tess_control_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.tctrl", root_dirs);
  resolve_includes(final_tess_control_shader_code, root_dirs);

  return final_tess_control_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_tess_evaluation_shader () const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_tess_eval_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.teval", root_dirs);
  resolve_includes(final_tess_eval_shader_code, root_dirs);

  return final_tess_eval_shader_code;
}


////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_geometry_shader () const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_geom_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.geom", root_dirs);
  resolve_includes(final_geom_shader_code, root_dirs);

  return final_geom_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSUberShader::_final_fragment_shader () const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_fragment_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.frag", root_dirs);
  resolve_includes(final_fragment_shader_code, root_dirs);

  return final_fragment_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
void NURBSUberShader::_insert_generic_per_vertex_code(std::string& code) const
{
  // material specific uniforms
  string_utils::replace(code, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // output
  string_utils::replace(code, "@output_definition",
    vshader_factory_->get_output_mapping().get_gbuffer_output_definition(false, true));

  // print material specific methods
  string_utils::replace(code, "@material_methods",
    UberShader::print_material_methods(*vshader_factory_));

  // print main switch(es)
  string_utils::replace(code, "@material_switch",
    UberShader::print_material_switch(*vshader_factory_));
}

////////////////////////////////////////////////////////////////////////////////
void NURBSUberShader::_insert_generic_per_fragment_code(std::string& code) const
{
  // input from vertex shader
  string_utils::replace(code, "@input_definition",
    vshader_factory_->get_output_mapping().get_gbuffer_output_definition(true, true));

  // material specific uniforms
  string_utils::replace(code, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // outputs
  string_utils::replace(code, "@output_definition",
    get_gbuffer_mapping()->get_gbuffer_output_definition(false, false));

  // print material specific methods
  string_utils::replace(code, "@material_methods",
    UberShader::print_material_methods(*fshader_factory_));

  // print main switch(es)
  string_utils::replace(code, "@material_switch",
    UberShader::print_material_switch(*fshader_factory_));
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
