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
#include <gua/renderer/PLODRenderer.hpp>

// guacamole headers
#include <gua/renderer/PLODResource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ResourceFactory.hpp>

#include <gua/node/PLODNode.hpp>
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/renderer/View.hpp>

#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>
#include <scm/gl_core/shader_objects.h>

// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>
#include <pbr/ren/camera.h>
#include <pbr/ren/policy.h>
#include <pbr/ren/model_database.h>
#include <pbr/ren/cut_database.h>
#include <pbr/ren/controller.h>
#include <pbr/ren/raw_point_cloud.h>
#include <boost/assign/list_of.hpp>

namespace gua {


  std::vector<math::vec3> PLODRenderer::_get_frustum_corners_vs(gua::Frustum const& frustum) const{
    std::vector<math::vec4> tmp(8);
    std::vector<math::vec3> result(8);

    auto inverse_transform(scm::math::inverse(frustum.get_projection() ));

    tmp[0] = inverse_transform * math::vec4(-1, -1, -1, 1);
    tmp[1] = inverse_transform * math::vec4(-1, -1,  1, 1);
    tmp[2] = inverse_transform * math::vec4(-1,  1, -1, 1);
    tmp[3] = inverse_transform * math::vec4(-1,  1,  1, 1);
    tmp[4] = inverse_transform * math::vec4( 1, -1, -1, 1);
    tmp[5] = inverse_transform * math::vec4( 1, -1,  1, 1);
    tmp[6] = inverse_transform * math::vec4( 1,  1, -1, 1);
    tmp[7] = inverse_transform * math::vec4( 1,  1,  1, 1);

    for (int i(0); i<8; ++i) {
      result[i] = tmp[i]/tmp[i][3];
    }

    return result;
  }

  //////////////////////////////////////////////////////////////////////////////
  PLODRenderer::PLODRenderer() : shaders_loaded_(false),
                                 gpu_resources_already_created_(false),
                                 depth_pass_program_(nullptr),
                                 normalization_pass_program_(nullptr),
                                 current_rendertarget_width_(0),
                                 current_rendertarget_height_(0)
 {
    _load_shaders();
  }

  PLODRenderer::~PLODRenderer() {
//substitute this part later with shared ptr in pbr_lib
/*
    Logger::LOG_DEBUG << "memory cleanup...(1)" << std::endl;
    delete pbr::ren::Controller::GetInstance();

    Logger::LOG_DEBUG << "deleted controller" << std::endl;
    delete pbr::ren::ModelDatabase::GetInstance();

    Logger::LOG_DEBUG << "deleted model database" << std::endl;
    delete pbr::ren::Policy::GetInstance();

    Logger::LOG_DEBUG << "deleted memory query" << std::endl;
    delete pbr::ren::CutDatabase::GetInstance();

    Logger::LOG_DEBUG << "deleted cut database" << std::endl;
*/
  }

  //////////////////////////////////////////////////////////////////////////////
  void PLODRenderer::_load_shaders() {
    //create stages only with one thread!
    if(!shaders_loaded_) {

#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
      ResourceFactory factory;
      depth_pass_shader_stages_.clear();
      depth_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p01_depth.vert")) );
      depth_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p01_depth.geom")) );
      depth_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p01_depth.frag")));

      accumulation_pass_shader_stages_.clear();
      accumulation_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p02_accumulation.vert")));
      accumulation_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p02_accumulation.geom")));
      accumulation_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p02_accumulation.frag")));

      normalization_pass_shader_stages_.clear();
      normalization_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p03_normalization.vert")));
      normalization_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p03_normalization.frag")));

      shaders_loaded_ = true;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  void PLODRenderer::_initialize_depth_pass_program() {
    if(!depth_pass_program_) {
      auto new_program = std::make_shared<ShaderProgram>();
      new_program->set_shaders(depth_pass_shader_stages_);
      depth_pass_program_ = new_program;
    }
    assert(depth_pass_program_);
  }

  //////////////////////////////////////////////////////////////////////////////
  void PLODRenderer::_initialize_accumulation_pass_program(MaterialShader* material) {
    if(!accumulation_pass_programs_.count(material)) {
      auto program = std::make_shared<ShaderProgram>();

      auto smap = global_substitution_map_;
      for (const auto& i : material->generate_substitution_map()) {
        smap[i.first] = i.second;
      }

      program->set_shaders(accumulation_pass_shader_stages_, std::list<std::string>(), false, smap);
      accumulation_pass_programs_[material] = program;
    }
    assert(accumulation_pass_programs_.count(material));
  }

  //////////////////////////////////////////////////////////////////////////////
  void PLODRenderer::_initialize_normalization_pass_program() {
    if(!normalization_pass_program_) {
      auto new_program = std::make_shared<ShaderProgram>();
      new_program->set_shaders(normalization_pass_shader_stages_);
      normalization_pass_program_ = new_program;
    }
    assert(normalization_pass_program_);
  }

  ///////////////////////////////////////////////////////////////////////////////
  void PLODRenderer::_create_gpu_resources(gua::RenderContext const& ctx,
                                           scm::math::vec2ui const& render_target_dims,
             bool resize_resource_containers) {

    depth_pass_log_depth_result_ = ctx.render_device
      ->create_texture_2d(render_target_dims,
                          scm::gl::FORMAT_D24,
                          1, 1, 1);

    depth_pass_linear_depth_result_ = ctx.render_device
      ->create_texture_2d(render_target_dims,
                          scm::gl::FORMAT_R_32F,
                          1, 1, 1);
          
    accumulation_pass_color_result_ = ctx.render_device
      ->create_texture_2d(render_target_dims,
                          scm::gl::FORMAT_RGBA_16F,
                          1, 1, 1);

    accumulation_pass_normal_result_ = ctx.render_device
      ->create_texture_2d(render_target_dims,
                          scm::gl::FORMAT_RGBA_16F,
                          1, 1, 1);

    accumulation_pass_pbr_result_ = ctx.render_device
      ->create_texture_2d(render_target_dims,
                          scm::gl::FORMAT_RGB_32F,
                          1, 1, 1);

    depth_pass_result_fbo_ = ctx.render_device->create_frame_buffer();
    depth_pass_result_fbo_->clear_attachments();
    depth_pass_result_fbo_->attach_depth_stencil_buffer(depth_pass_log_depth_result_);
    depth_pass_result_fbo_->attach_color_buffer(0,
                                                depth_pass_linear_depth_result_);

    accumulation_pass_result_fbo_ = ctx.render_device->create_frame_buffer();
    accumulation_pass_result_fbo_->clear_attachments();
    accumulation_pass_result_fbo_->attach_color_buffer(0,
                                                       accumulation_pass_color_result_);
    accumulation_pass_result_fbo_->attach_color_buffer(1,
                                                       accumulation_pass_normal_result_);
    accumulation_pass_result_fbo_->attach_color_buffer(2,
                                                       accumulation_pass_pbr_result_);
    accumulation_pass_result_fbo_->attach_depth_stencil_buffer(depth_pass_log_depth_result_);

  
    nearest_sampler_state_ = ctx.render_device
      ->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

    no_depth_test_depth_stencil_state_ = ctx.render_device
      ->create_depth_stencil_state(false, false, scm::gl::COMPARISON_ALWAYS);

    depth_test_without_writing_depth_stencil_state_ = ctx.render_device
      ->create_depth_stencil_state(true, false, scm::gl::COMPARISON_LESS_EQUAL);

    color_accumulation_state_ = ctx.render_device->create_blend_state(true,
                                                                      scm::gl::FUNC_ONE,
                                                                      scm::gl::FUNC_ONE,
                                                                      scm::gl::FUNC_ONE,
                                                                      scm::gl::FUNC_ONE,
                                                                      scm::gl::EQ_FUNC_ADD,
                                                                      scm::gl::EQ_FUNC_ADD);

    no_backface_culling_rasterizer_state_ = ctx.render_device
      ->create_rasterizer_state(scm::gl::FILL_SOLID,
                                scm::gl::CULL_NONE,
                                scm::gl::ORIENT_CCW,
                                false,
                                false,
                                0.0,
                                false,
                                false,
                                scm::gl::point_raster_state(false));


    fullscreen_quad_.reset(new scm::gl::quad_geometry(ctx.render_device, 
                                                      scm::math::vec2(-1.0f, -1.0f), scm::math::vec2(1.0f, 1.0f )));

    //invalidation before first write
    previous_frame_count_ = UINT_MAX;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  pbr::context_t PLODRenderer::_register_context_in_cut_update(gua::RenderContext const& ctx) {
    //pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();

    //if( previous_frame_count_ != ctx.framecount ) {
      previous_frame_count_ = ctx.framecount;

      pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
      controller->ResetSystem();
      pbr::context_t context_id = controller->DeduceContextId( (size_t)(&ctx) );
      controller->Dispatch(context_id , ctx.render_device);

      return context_id;
    //}
    //else {
    //  return controller->DeduceContextId( (size_t)(&ctx.id));
    //}
  }

  std::shared_ptr<ShaderProgram> PLODRenderer::_get_material_program(MaterialShader* material,
                                                                     std::shared_ptr<ShaderProgram> const& current_program,
                                                                     bool& program_changed) {
    auto shader_iterator = accumulation_pass_programs_.find(material);
    if (shader_iterator == accumulation_pass_programs_.end()) {
      try {
        _initialize_accumulation_pass_program(material);
        program_changed = true;
        return accumulation_pass_programs_.at(material);
      } 
      catch (std::exception& e) {
        Logger::LOG_WARNING << "PLODPass::_get_material_program(): Cannot create material for accumulation pass program: " << e.what() << std::endl;
        return std::shared_ptr<ShaderProgram>();
      }
    }
    else {
      if (current_program == shader_iterator->second) {
        program_changed = false;
        return current_program;
      }
      else {
        program_changed = true;
        return shader_iterator->second;
      }
    }

  }

  ///////////////////////////////////////////////////////////////////////////////
  void PLODRenderer::render(gua::Pipeline& pipe, PipelinePassDescription const& desc) {

    auto sorted_objects(pipe.get_scene().nodes.find(std::type_index(typeid(node::PLODNode))));

    if(sorted_objects != pipe.get_scene().nodes.end() && sorted_objects->second.size() > 0) {
      
      std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b) {
        return reinterpret_cast<node::PLODNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::PLODNode*>(b)->get_material()->get_shader();
      });

      RenderContext const& ctx(pipe.get_context());
   

      scm::math::vec2ui const& render_target_dims = pipe.get_camera().config.get_resolution();
      //allocate GPU resources if necessary
      {

  bool resize_resource_containers = false;

  if(gpu_resources_already_created_ == false || current_rendertarget_width_ != render_target_dims[0] || current_rendertarget_height_ != render_target_dims[1]) {
          current_rendertarget_width_ = render_target_dims[0];
          current_rendertarget_height_ = render_target_dims[1];
          _create_gpu_resources(ctx, render_target_dims, resize_resource_containers);
          gpu_resources_already_created_ = true;
        }


      }

      //register context for cut-update   
       pbr::context_t context_id = _register_context_in_cut_update(ctx);
       
       //clear render targets of active FBOs
       //ctx.render_context
       //  ->clear_depth_stencil_buffer(depth_pass_result_fbo_);
       ctx.render_context
         ->clear_color_buffer(depth_pass_result_fbo_, 0, scm::math::vec4(1.0f, 1.0f, 1.0f, 1.0f));

       ctx.render_context
         ->clear_color_buffer(accumulation_pass_result_fbo_,
                              0,
                              scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));
       ctx.render_context
         ->clear_color_buffer(accumulation_pass_result_fbo_,
                              1,
                              scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));

       ctx.render_context
         ->clear_color_buffer(accumulation_pass_result_fbo_,
                              2,
                              scm::math::vec4f(0.0f, 0.0f, 0.0f));

     pipe.get_gbuffer().set_viewport(ctx);

     //determine frustum parameters
     Frustum const& frustum = pipe.get_scene().frustum;
     std::vector<math::vec3> frustum_corner_values = _get_frustum_corners_vs(frustum);
     float top_minus_bottom = scm::math::length((frustum_corner_values[2]) - 
                                                (frustum_corner_values[0]));

     float height_divided_by_top_minus_bottom =
         pipe.get_gbuffer().get_height() / (top_minus_bottom);

     float near_plane_value = frustum.get_clip_near();
     float far_plane_value = frustum.get_clip_far();


      if(!depth_pass_program_) {
        _initialize_depth_pass_program();
        save_to_file(*depth_pass_program_, ".", "depth_pass_debug");
      }

     //create pbr camera out of gua camera values
     pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
     pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();
     pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();

     pbr::view_t view_id = controller->DeduceViewId(context_id, (size_t)(&pipe));

     pbr::ren::Camera cut_update_cam(
       view_id, near_plane_value, frustum.get_view(), frustum.get_projection());

     cuts->SendCamera(context_id, view_id, cut_update_cam);
     cuts->SendHeightDividedByTopMinusBottom(context_id, view_id, height_divided_by_top_minus_bottom);

      std::unordered_map<pbr::model_t, std::unordered_set<pbr::node_t> > nodes_out_of_frustum_per_model;

     auto& gua_depth_buffer = pipe.get_gbuffer().get_current_depth_buffer()->get_buffer(ctx);


     MaterialShader* current_material(nullptr);
     std::shared_ptr<ShaderProgram> current_material_program;


     //depth pass 
     {
       std::shared_ptr<scm::gl::context_all_guard> context_guard = std::make_shared<scm::gl::context_all_guard>(ctx.render_context);

       ctx.render_context
         ->set_rasterizer_state(no_backface_culling_rasterizer_state_);
 
       depth_pass_result_fbo_->attach_depth_stencil_buffer( gua_depth_buffer );

       ctx.render_context
         ->set_frame_buffer(depth_pass_result_fbo_);
 
       if(!depth_pass_program_) {
         std::cout << "Depth program not instanciated\n";
       }

       depth_pass_program_->use(ctx);

       nodes_out_of_frustum_per_model.clear();

      //loop through all models and render depth pass
      for (auto const& object : sorted_objects->second) { 

        auto plod_node(reinterpret_cast<node::PLODNode*>(object));

        pbr::model_t model_id = controller->DeduceModelId( plod_node->get_geometry_description());

        auto const& scm_model_matrix = plod_node->get_cached_world_transform();

        auto const& scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));
 

        cuts->SendTransform(context_id, model_id, scm_model_matrix);
        cuts->SendRendered(context_id, model_id);
        cuts->SendImportance(context_id, model_id, plod_node->get_importance());

        pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
        std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();

        //perform frustum culling 
        pbr::ren::KdnTree const* kdn_tree = database->GetModel(model_id)->kdn_tree();       
        scm::gl::frustum const& culling_frustum =
          cut_update_cam.GetFrustumByModel(scm_model_matrix);

        std::vector<scm::gl::boxf> const& model_bounding_boxes = kdn_tree->bounding_boxes();
        
        std::unordered_set<pbr::node_t>& nodes_out_of_frustum = nodes_out_of_frustum_per_model[model_id]; 

        for(const auto& n : node_list) {
          if(culling_frustum.classify(model_bounding_boxes[n.node_id_]) == 1) {
            nodes_out_of_frustum.insert(n.node_id_);
          }
        }


  UniformValue model_mat(scm_model_matrix);
  UniformValue normal_mat(scm_normal_matrix);

        depth_pass_program_->apply_uniform(ctx,
                                           "gua_model_matrix",
                                           model_mat);

        depth_pass_program_->apply_uniform(ctx,
                                           "gua_normal_matrix",
                                           normal_mat);

        float radius_importance_scaling = plod_node->get_importance();      
   
        depth_pass_program_->apply_uniform(ctx, 
                                          "radius_importance_scaling",
                                           radius_importance_scaling);
        
        ctx.render_context->apply();

        auto plod_resource = plod_node->get_geometry();

  if(plod_resource && depth_pass_program_) {

          plod_resource->draw(ctx,
                             context_id,
                             view_id,
                             model_id,
                             controller->GetContextMemory(context_id, ctx.render_device),
                             nodes_out_of_frustum);
        }
        else {
          Logger::LOG_WARNING << "PLODRenderer::render(): Cannot find ressources for node: " << plod_node->get_name() << std::endl;
        }
      }
     
       depth_pass_program_->use(ctx);
     }
     
    /*  if(!accumulation_pass_programs_.count()) {
        _initialize_accumulation_pass_program();
        save_to_file(*accumulation_pass_program_,".", "accumulation_pass_debug");
      }
    */

     //accumulation pass 
     {
       std::shared_ptr<scm::gl::context_all_guard> context_guard = std::make_shared<scm::gl::context_all_guard>(ctx.render_context);

       ctx.render_context
         ->set_rasterizer_state(no_backface_culling_rasterizer_state_);
 
       ctx.render_context
          ->set_depth_stencil_state(depth_test_without_writing_depth_stencil_state_);
       //ctx.render_context
           //->set_depth_stencil_state(no_depth_test_depth_stencil_state_);

       ctx.render_context->set_blend_state(color_accumulation_state_);

       ctx.render_context
         ->set_frame_buffer(accumulation_pass_result_fbo_);

       accumulation_pass_result_fbo_->attach_depth_stencil_buffer(gua_depth_buffer );
 
      /* if(!accumulation_pass_program_) {
         std::cout << "Accumulation program not instanciated\n";
       }
      */


      int view_id(pipe.get_camera().config.get_view_id());
      bool program_changed = false;
      //loop through all models and render accumulation pass
      for (auto const& object : sorted_objects->second) { 

        auto plod_node(reinterpret_cast<node::PLODNode*>(object));

        pbr::model_t model_id = controller->DeduceModelId( plod_node->get_geometry_description());
        

        
        pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
        std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();

        current_material = plod_node->get_material()->get_shader();

        current_material_program = _get_material_program(current_material, 
                                                       current_material_program, 
                                                       program_changed);

        ctx.render_context->apply();

        auto plod_resource = plod_node->get_geometry();
       
        //retrieve frustum culling results      
        std::unordered_set<pbr::node_t>& nodes_out_of_frustum = nodes_out_of_frustum_per_model[model_id]; 

  if(plod_resource && current_material_program) {

        if(program_changed) {
          current_material_program->unuse(ctx);
          current_material_program->use(ctx);

          ctx.render_context
            ->bind_texture(depth_pass_linear_depth_result_, nearest_sampler_state_, 0);

          current_material_program->apply_uniform(ctx,
                                                   "p01_linear_depth_texture", 0);
        }

        auto const& scm_model_matrix = plod_node->get_cached_world_transform();
        auto const& scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));

        UniformValue model_mat(scm_model_matrix);
        UniformValue normal_mat(scm_normal_matrix);

        current_material_program->apply_uniform(ctx,
                                           "gua_model_matrix",
                                           scm_model_matrix);

        current_material_program->apply_uniform(ctx,
                                           "gua_normal_matrix",
                                           normal_mat);

        float radius_importance_scaling = plod_node->get_importance();               
   
        current_material_program->apply_uniform(ctx, 
                                          "radius_importance_scaling",
                                           radius_importance_scaling);

          plod_node->get_material()->apply_uniforms(ctx, current_material_program.get(), view_id);

          plod_resource->draw(ctx,
                             context_id,
                             view_id,
                             model_id,
                             controller->GetContextMemory(context_id, ctx.render_device),
                             nodes_out_of_frustum);

          program_changed = false;
        }
        else {
          Logger::LOG_WARNING << "PLODRenderer::render(): Cannot find ressources for node: " << plod_node->get_name() << std::endl;
        }
      }
     
       current_material_program->unuse(ctx);
     }

      if(!normalization_pass_program_) {
        _initialize_normalization_pass_program();
        save_to_file(*normalization_pass_program_,".", "normalization_pass_debug");
      }


     bool writes_only_color_buffer = false;
     pipe.get_gbuffer().bind(ctx, writes_only_color_buffer);

     //normalization pass 
     {
       std::shared_ptr<scm::gl::context_all_guard> context_guard = std::make_shared<scm::gl::context_all_guard>(ctx.render_context);

       if(!normalization_pass_program_) {
         std::cout << "Normalization program not instanciated\n";
       }

       normalization_pass_program_->use(ctx);

       ctx.render_context
          ->set_depth_stencil_state( no_depth_test_depth_stencil_state_ );

       //ctx.render_context
       //  ->bind_texture(gua_depth_buffer, linear_sampler_state_, 0);
       //normalization_pass_program_->apply_uniform(ctx,
       //                                          "p01_depth_texture", 0);

       ctx.render_context
         ->bind_texture(accumulation_pass_color_result_, nearest_sampler_state_, 0);
       normalization_pass_program_->apply_uniform(ctx,
                                                 "p02_color_texture", 0);

       ctx.render_context
         ->bind_texture(accumulation_pass_normal_result_, nearest_sampler_state_, 1);
       normalization_pass_program_->apply_uniform(ctx,
                                                 "p02_normal_texture", 1);

       ctx.render_context
         ->bind_texture(accumulation_pass_pbr_result_, nearest_sampler_state_, 2);
       normalization_pass_program_->apply_uniform(ctx,
                                                 "p02_pbr_texture", 2);
                                             
       ctx.render_context->apply();

       fullscreen_quad_->draw(ctx.render_context);
       normalization_pass_program_->unuse(ctx);
     }

   
       pipe.get_gbuffer().unbind(ctx);
    }
  }



}
