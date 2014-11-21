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
#include <gua/renderer/NURBSRenderer.hpp>

// guacamole headers
#include <gua/renderer/NURBSRessource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/node/NURBSNode.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>
#include <gpucast/core/config.hpp>

#include <scm/gl_core/shader_objects.h>

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
        std::string full_path(path);
        std::ifstream ifstr(full_path.c_str(), std::ios::in);

        if (ifstr.good()) {
          return std::string(std::istreambuf_iterator<char>(ifstr), std::istreambuf_iterator<char>());
        }

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


#if 0

  TriMeshPassDescription::TriMeshPassDescription()
    : PipelinePassDescription() {
    vertex_shader_ = ""; // "shaders/tri_mesh_shader.vert";
    fragment_shader_ = ""; // "shaders/tri_mesh_shader.frag";

    needs_color_buffer_as_input_ = false;
    writes_only_color_buffer_ = false;
    doClear_ = false;
    rendermode_ = RenderMode::Custom;

    std::shared_ptr<scm::gl::buffer_ptr> material_uniform_storage_buffer = std::make_shared<scm::gl::buffer_ptr>(nullptr);
    auto vertex_shader = Resources::lookup_shader("shaders/tri_mesh_shader.vert");
    auto fragment_shader = Resources::lookup_shader("shaders/tri_mesh_shader.frag");
    process_ = [material_uniform_storage_buffer, vertex_shader, fragment_shader](
      PipelinePass&, PipelinePassDescription const&, Pipeline & pipe) {

      auto sorted_objects(pipe.get_scene().nodes.find(std::type_index(typeid(node::TriMeshNode))));

      if (sorted_objects != pipe.get_scene().nodes.end() && sorted_objects->second.size() > 0) {

        std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b){
          return reinterpret_cast<node::TriMeshNode*>(a)->get_material().get_shader() < reinterpret_cast<node::TriMeshNode*>(b)->get_material().get_shader();
        });

        RenderContext const& ctx(pipe.get_context());

        bool writes_only_color_buffer = false;
        pipe.get_gbuffer().bind(ctx, writes_only_color_buffer);
        pipe.get_gbuffer().set_viewport(ctx);
        int view_id(pipe.get_camera().config.get_view_id());

        MaterialShader* current_material(nullptr);
        ShaderProgram*  current_shader(nullptr);

        // loop through all objects, sorted by material ----------------------------
        for (auto const& object : sorted_objects->second) {

          auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(object));

          if (current_material != tri_mesh_node->get_material().get_shader()) {
            current_material = tri_mesh_node->get_material().get_shader();
            if (current_material) {
              current_shader = current_material->get_shader(ctx, typeid(*tri_mesh_node->get_geometry()), vertex_shader, fragment_shader);
            }
            else {
              Logger::LOG_WARNING << "TriMeshPass::process(): Cannot find material: " << tri_mesh_node->get_material().get_shader_name() << std::endl;
            }
            if (current_shader) {
              current_shader->use(ctx);
              ctx.render_context->apply();
            }
          }

          if (current_shader && tri_mesh_node->get_geometry()) {
            UniformValue model_mat(tri_mesh_node->get_cached_world_transform());
            UniformValue normal_mat(scm::math::transpose(scm::math::inverse(tri_mesh_node->get_cached_world_transform())));

            current_shader->apply_uniform(ctx, "gua_model_matrix", model_mat);
            current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);

            for (auto const& overwrite : tri_mesh_node->get_material().get_uniforms()) {
              current_shader->apply_uniform(ctx, overwrite.first, overwrite.second.get(view_id));
            }

            ctx.render_context->apply_program();

            tri_mesh_node->get_geometry()->draw(ctx);
          }
        }

        pipe.get_gbuffer().unbind(ctx);
      }
    };
  }

  ////////////////////////////////////////////////////////////////////////////////

  PipelinePassDescription* TriMeshPassDescription::make_copy() const {
    return new TriMeshPassDescription(*this);
  }


#endif


  ////////////////////////////////////////////////////////////////////////////////
  NURBSRenderer::NURBSRenderer()
  {
    _load_shaders();
  }

  ////////////////////////////////////////////////////////////////////////////////
  NURBSRenderer::~NURBSRenderer()
  {}

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSRenderer::_load_shaders()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // create stages only with one thread!
    if (!shaders_loaded_)
    {
      pre_tesselation_interleaved_stream_capture_.clear();

      pre_tesselation_shader_stages_[scm::gl::STAGE_VERTEX_SHADER] = _transform_feedback_vertex_shader();
      pre_tesselation_shader_stages_[scm::gl::STAGE_TESS_CONTROL_SHADER] = _transform_feedback_tess_control_shader();
      pre_tesselation_shader_stages_[scm::gl::STAGE_TESS_EVALUATION_SHADER] = _transform_feedback_tess_evaluation_shader();
      pre_tesselation_shader_stages_[scm::gl::STAGE_GEOMETRY_SHADER] = _transform_feedback_geometry_shader();

      pre_tesselation_interleaved_stream_capture_.push_back("xfb_position");
      pre_tesselation_interleaved_stream_capture_.push_back("xfb_index");
      pre_tesselation_interleaved_stream_capture_.push_back("xfb_tesscoord");

      tesselation_shader_stages_[scm::gl::STAGE_VERTEX_SHADER] = _final_vertex_shader();
      tesselation_shader_stages_[scm::gl::STAGE_TESS_CONTROL_SHADER] = _final_tess_control_shader();
      tesselation_shader_stages_[scm::gl::STAGE_TESS_EVALUATION_SHADER] = _final_tess_evaluation_shader();
      tesselation_shader_stages_[scm::gl::STAGE_GEOMETRY_SHADER] = _final_geometry_shader();
      tesselation_shader_stages_[scm::gl::STAGE_FRAGMENT_SHADER] = _final_fragment_shader();

      raycasting_shader_stages_[scm::gl::STAGE_VERTEX_SHADER] = _raycast_vertex_shader();
      raycasting_shader_stages_[scm::gl::STAGE_FRAGMENT_SHADER] = _raycast_fragment_shader();

      shaders_loaded_ = true;
    }
    lock.release();
  }


////////////////////////////////////////////////////////////////////////////////
  std::string NURBSRenderer::_transform_feedback_vertex_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string vertex_shadercode = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/pre_tesselation.vert", root_dirs);
  resolve_includes(vertex_shadercode, root_dirs);

  return vertex_shadercode;
}

////////////////////////////////////////////////////////////////////////////////
  std::string NURBSRenderer::_transform_feedback_tess_control_shader() const
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
std::string NURBSRenderer::_transform_feedback_tess_evaluation_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string tess_eval_shadercode = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/pre_tesselation.teval", root_dirs);
  resolve_includes(tess_eval_shadercode, root_dirs);

  return tess_eval_shadercode;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSRenderer::_transform_feedback_geometry_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string geometry_shadercode = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/pre_tesselation.geom", root_dirs);
  resolve_includes(geometry_shadercode, root_dirs);

  return geometry_shadercode;
}


////////////////////////////////////////////////////////////////////////////////
std::string NURBSRenderer::_final_vertex_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_vertex_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.vert", root_dirs);
  resolve_includes(final_vertex_shader_code, root_dirs);

  return final_vertex_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSRenderer::_final_tess_control_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_tess_control_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.tctrl", root_dirs);
  resolve_includes(final_tess_control_shader_code, root_dirs);

  return final_tess_control_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSRenderer::_final_tess_evaluation_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_tess_eval_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.teval", root_dirs);
  resolve_includes(final_tess_eval_shader_code, root_dirs);

  return final_tess_eval_shader_code;
}


////////////////////////////////////////////////////////////////////////////////
std::string NURBSRenderer::_final_geometry_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_geom_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.geom", root_dirs);
  resolve_includes(final_geom_shader_code, root_dirs);

  return final_geom_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSRenderer::_final_fragment_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string final_fragment_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/final_tesselation.frag", root_dirs);
  resolve_includes(final_fragment_shader_code, root_dirs);

  return final_fragment_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSRenderer::_raycast_vertex_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string raycast_vertex_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/ray_casting.vert", root_dirs);
  resolve_includes(raycast_vertex_shader_code, root_dirs);

  return raycast_vertex_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
std::string NURBSRenderer::_raycast_fragment_shader() const
{
  std::vector<std::string> root_dirs = { GUACAMOLE_INSTALL_DIR, GPUCAST_INSTALL_DIR };

  std::string raycast_fragment_shader_code = read_shader_file("resources/shaders/uber_shaders/gbuffer/nurbs/ray_casting.frag", root_dirs);
  resolve_includes(raycast_fragment_shader_code, root_dirs);

  return raycast_fragment_shader_code;
}

////////////////////////////////////////////////////////////////////////////////
void NURBSRenderer::_insert_generic_per_vertex_code(std::string& code) const
{
#if 0
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
#endif
}

////////////////////////////////////////////////////////////////////////////////
void NURBSPassDescription::_insert_generic_per_fragment_code(std::string& code) const
{
#if 0
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
#endif
}


////////////////////////////////////////////////////////////////////////////////

void NURBSRenderer::render(Pipeline& pipe)
{
#if 0
  auto geometry = std::static_pointer_cast<NURBSRessource>(GeometryDatabase::instance()->lookup(filename));
  auto material = MaterialDatabase::instance()->lookup(material_name);

  //set_uniform(ctx, 8, "gua_max_tesselation");
  set_uniform(ctx, material->get_id(), "gua_material_id");
  set_uniform(ctx, model_matrix, "gua_model_matrix");
  set_uniform(ctx, normal_matrix, "gua_normal_matrix");
  

  if ( geometry->raycasting_enabled() ) 
  {
    get_program(raycasting)->set_uniform(ctx, frustum.get_clip_near(), "nearplane");
    get_program(raycasting)->set_uniform(ctx, frustum.get_clip_near(), "farplane");

    get_program(raycasting)->use(ctx);
    {
      ctx.render_context->apply();
      geometry->draw(ctx);
    }
    get_program(raycasting)->unuse(ctx);
  } else {
  #ifdef DEBUG_XFB_OUTPUT
    scm::gl::transform_feedback_statistics_query_ptr q = ctx
      .render_device->create_transform_feedback_statistics_query(0);
    ctx.render_context->begin_query(q);
#endif

    // pre-tesselate if necessary
    get_program(tesselation_pre_pass)->use(ctx);
    {
      ctx.render_context->apply();
      geometry->predraw(ctx);
    }
    get_program(tesselation_pre_pass)->unuse(ctx);

#ifdef DEBUG_XFB_OUTPUT
    ctx.render_context->end_query(q);
    ctx.render_context->collect_query_results(q);
    std::cout << q->result()._primitives_generated << " , "
      << q->result()._primitives_written << std::endl;
#endif

    // invoke tesselation/trim shader for adaptive nurbs rendering
    get_program(tesselation_final_pass)->use(ctx);
    {
      ctx.render_context->apply();
      geometry->draw(ctx);
    }
    get_program(tesselation_final_pass)->unuse(ctx);
  }
#endif

}

} // namespace gua
