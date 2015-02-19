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
#include <gua/renderer/NURBSResource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ResourceFactory.hpp>

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
    : pre_tesselation_program_(nullptr),
      factory_()
  {
    factory_.add_search_path(std::string(GPUCAST_INSTALL_DIR));
    factory_.add_search_path(std::string(GPUCAST_INSTALL_DIR) + "/resources/");

    _load_shaders();
  }

  ////////////////////////////////////////////////////////////////////////////////
  NURBSRenderer::~NURBSRenderer()
  {}

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSRenderer::_load_shaders()
  {
    pre_tesselation_shader_stages_.clear();
    pre_tesselation_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory_.read_shader_file("resources/shaders/nurbs/pre_tesselation.vert")));
    pre_tesselation_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_TESS_CONTROL_SHADER, factory_.read_shader_file("resources/shaders/nurbs/pre_tesselation.tctrl")));
    pre_tesselation_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_TESS_EVALUATION_SHADER, factory_.read_shader_file("resources/shaders/nurbs/pre_tesselation.teval")));
    pre_tesselation_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory_.read_shader_file("resources/shaders/nurbs/pre_tesselation.geom")));

    pre_tesselation_interleaved_stream_capture_.clear();
    pre_tesselation_interleaved_stream_capture_.push_back("xfb_position");
    pre_tesselation_interleaved_stream_capture_.push_back("xfb_index");
    pre_tesselation_interleaved_stream_capture_.push_back("xfb_tesscoord");

    tesselation_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory_.read_shader_file("resources/shaders/nurbs/final_tesselation.vert")));
    tesselation_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_TESS_CONTROL_SHADER, factory_.read_shader_file("resources/shaders/nurbs/final_tesselation.tctrl")));
    tesselation_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_TESS_EVALUATION_SHADER, factory_.read_shader_file("resources/shaders/nurbs/final_tesselation.teval")));
    tesselation_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory_.read_shader_file("resources/shaders/nurbs/final_tesselation.geom")));
    tesselation_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory_.read_shader_file("resources/shaders/nurbs/final_tesselation.frag")));

    raycasting_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory_.read_shader_file("resources/shaders/nurbs/ray_casting.vert")));
    raycasting_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory_.read_shader_file("resources/shaders/nurbs/ray_casting.frag")));
  }

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSRenderer::_initialize_pre_tesselation_program() 
  {
    if (!pre_tesselation_program_)
    {
      auto new_program = std::make_shared<ShaderProgram>();
      new_program->set_shaders(pre_tesselation_shader_stages_, pre_tesselation_interleaved_stream_capture_, true);
      pre_tesselation_program_ = new_program;
      save_to_file(*pre_tesselation_program_, ".", "pre_tesselation_program");
    }
    assert(pre_tesselation_program_);
  }

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSRenderer::_initialize_tesselation_program(MaterialShader* material)
  {
    if (!tesselation_programs_.count(material))
    {
      auto program = std::make_shared<ShaderProgram>();

      auto smap    = global_substitution_map_;
      for (const auto& i : material->generate_substitution_map()) {
        smap[i.first] = i.second;
      }

      program->set_shaders(tesselation_shader_stages_, std::list<std::string>(), false, smap);
      tesselation_programs_[material] = program;
    }
    assert(tesselation_programs_.count(material));
  }

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSRenderer::_initialize_raycasting_program(MaterialShader* material)
  {
    if (!raycasting_programs_.count(material))
    {
      auto program = std::make_shared<ShaderProgram>();

      auto smap = global_substitution_map_;
      for (const auto& i : material->generate_substitution_map()) {
        smap[i.first] = i.second;
      }

      program->set_shaders(raycasting_shader_stages_, std::list<std::string>(), false, smap);
      raycasting_programs_[material] = program;
    }
    assert(raycasting_programs_.count(material));
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<ShaderProgram> NURBSRenderer::_get_material_program(MaterialShader* material,
                                                                      std::shared_ptr<ShaderProgram> const& current_program,
                                                                      bool raycasting,
                                                                      bool& program_changed)
  {
    if (raycasting) {
      auto shader_iterator = raycasting_programs_.find(material);
      if (shader_iterator == raycasting_programs_.end())
      {
        try {
          _initialize_raycasting_program(material);
          program_changed = true;
          return raycasting_programs_.at(material);
        }
        catch (std::exception& e) {
          Logger::LOG_WARNING << "NURBSPass::_get_material_program(): Cannot create material for raycasting program: " << e.what() << std::endl; 
          return std::shared_ptr<ShaderProgram>();
        }
      }
      else {
        if (current_program == shader_iterator->second)
        {
          program_changed = false;
          return current_program;
        }
        else {
          program_changed = true;
          return shader_iterator->second;
        }
      }
    } else {
      auto shader_iterator = tesselation_programs_.find(material);
      if (shader_iterator == tesselation_programs_.end())
      {
        try {
          _initialize_tesselation_program(material);
          program_changed = true;
          return tesselation_programs_.at(material);
        }
        catch (std::exception& e) {
          Logger::LOG_WARNING << "NURBSPass::_get_material_program(): Cannot create material for tesselation program: " << e.what() << std::endl;
          return std::shared_ptr<ShaderProgram>();
        }
      } else {
        if (current_program == shader_iterator->second)
        {
          program_changed = false;
          return current_program;
        }
        else {
          program_changed = true;
          return shader_iterator->second;
        }
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSRenderer::_reset()
  {
    pre_tesselation_program_.reset();
    tesselation_programs_.clear();
    raycasting_programs_.clear();
  }


////////////////////////////////////////////////////////////////////////////////

void NURBSRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{
  auto sorted_objects(pipe.get_scene().nodes.find(std::type_index(typeid(node::NURBSNode))));

  if (sorted_objects != pipe.get_scene().nodes.end() && sorted_objects->second.size() > 0) {

    std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b){
      return reinterpret_cast<node::NURBSNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::NURBSNode*>(b)->get_material()->get_shader();
    });

    RenderContext const& ctx(pipe.get_context());

    bool writes_only_color_buffer = false;
    pipe.get_gbuffer().bind(ctx, writes_only_color_buffer);
    pipe.get_gbuffer().set_viewport(ctx);

    int view_id(pipe.get_camera().config.get_view_id());
    
    MaterialShader* current_material(nullptr);
    std::shared_ptr<ShaderProgram> current_material_program;

    if (!pre_tesselation_program_) {
      _initialize_pre_tesselation_program();
      //pre_tesselation_program_->save_to_file(".", "pre_tesselation");
    }
    bool program_changed = false;

    // loop through all objects, sorted by material ----------------------------
    for (auto const& object : sorted_objects->second) 
    {
      auto nurbs_node(reinterpret_cast<node::NURBSNode*>(object));
      current_material = nurbs_node->get_material()->get_shader();

      current_material_program = _get_material_program(current_material, 
                                                       current_material_program, 
                                                       nurbs_node->raycasting(), 
                                                       program_changed);

      UniformValue model_mat(nurbs_node->get_cached_world_transform());
      UniformValue normal_mat(scm::math::transpose(scm::math::inverse(nurbs_node->get_cached_world_transform())));

      auto nurbs_ressource = nurbs_node->get_geometry();

      if (nurbs_ressource && pre_tesselation_program_ && current_material_program) {

        if (nurbs_node->raycasting())
        {
          // render using raycasting
          current_material_program->use(ctx);
          save_to_file(*current_material_program, ".", "raycasting");
          {
            current_material_program->apply_uniform(ctx, "gua_model_matrix", model_mat);
            current_material_program->apply_uniform(ctx, "gua_normal_matrix", normal_mat);
          
            current_material_program->apply_uniform(ctx, "nearplane", pipe.get_camera().config.get_near_clip());
            current_material_program->apply_uniform(ctx, "farplane", pipe.get_camera().config.get_far_clip());
          
            current_material_program->set_uniform(ctx, math::vec2i(pipe.get_gbuffer().get_width(),
              pipe.get_gbuffer().get_height()),
              "gua_resolution"); 
          
            nurbs_node->get_material()->apply_uniforms(ctx, current_material_program.get(), view_id);
          
            nurbs_ressource->draw(ctx, true, nurbs_node->render_backfaces());
          }
          current_material_program->unuse(ctx);
        } else {
         
#define DEBUG_XFB_OUTPUT 0
#if DEBUG_XFB_OUTPUT
        scm::gl::transform_feedback_statistics_query_ptr q = ctx
          .render_device->create_transform_feedback_statistics_query(0);
        ctx.render_context->begin_query(q);
#endif
          // render using two-pass tesselation approach
          pre_tesselation_program_->use(ctx);
          {
            pre_tesselation_program_->apply_uniform(ctx, "gua_model_matrix", model_mat);
            pre_tesselation_program_->apply_uniform(ctx, "gua_normal_matrix", normal_mat);

            ctx.render_context->apply();
            nurbs_ressource->predraw(ctx, nurbs_node->render_backfaces());
          }
          pre_tesselation_program_->unuse(ctx);

  #if DEBUG_XFB_OUTPUT
          ctx.render_context->end_query(q);
          ctx.render_context->collect_query_results(q);
          std::cout << q->result()._primitives_generated << " , "
            << q->result()._primitives_written << std::endl;
  #endif

          current_material_program->use(ctx);
          save_to_file(*current_material_program, ".", "tesselation");
          {
            nurbs_node->get_material()->apply_uniforms(ctx, current_material_program.get(), view_id);

            current_material_program->apply_uniform(ctx, "gua_model_matrix", model_mat);
            current_material_program->apply_uniform(ctx, "gua_normal_matrix", normal_mat);

            current_material_program->set_uniform(ctx, math::vec2i(pipe.get_gbuffer().get_width(),
              pipe.get_gbuffer().get_height()),
              "gua_resolution");

            ctx.render_context->apply();
            nurbs_ressource->draw(ctx, false, nurbs_node->render_backfaces());
          }
          current_material_program->unuse(ctx);
        }
      }
      else {
        Logger::LOG_WARNING << "NURBSPass::render(): Cannot find ressources for node: " << nurbs_node->get_name() << std::endl;
      }
      
    }

    pipe.get_gbuffer().unbind(ctx);
  }
}

} // namespace gua
