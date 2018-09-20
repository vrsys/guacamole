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
#include <gua/virtual_texturing/DeferredVirtualTexturingRenderer.hpp>
#include <gua/virtual_texturing/DeferredVirtualTexturingPass.hpp>
#include <gua/virtual_texturing/VirtualTexture2D.hpp>

// guacamole headers
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ResourceFactory.hpp>

#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/renderer/View.hpp>

#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>

// schism headers
#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/render_device/context.h>


// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>

// lamure headers

#include <lamure/vt/VTConfig.h>

#include <lamure/vt/common.h>
#include <lamure/vt/ren/CutUpdate.h>
#include <lamure/vt/ren/CutDatabase.h>

#include <boost/assign/list_of.hpp>
#include <typeinfo>

namespace gua {


  //////////////////////////////////////////////////////////////////////////////
  DeferredVirtualTexturingRenderer::DeferredVirtualTexturingRenderer(RenderContext const& ctx, SubstitutionMap const& smap)
  : screen_space_virtual_texturing_shader_program_(nullptr),
    blit_vt_color_to_gbuffer_program_(nullptr),
    fullscreen_quad_(nullptr)
  {

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  ResourceFactory factory;
  std::string v_shader_screen_space_virtual_texturing = factory.read_shader_file("resources/shaders/common/fullscreen_quad.vert");
  std::string f_shader_screen_space_virtual_texturing = factory.read_shader_file("resources/shaders/screen_space_virtual_texturing.frag");

  std::string v_shader_blit_vt_color = factory.read_shader_file("resources/shaders/common/fullscreen_quad.vert");
  std::string f_shader_blit_vt_color = factory.read_shader_file("resources/shaders/blit_virtual_texturing_colors.frag");
#else  
  std::string v_shader_screen_space_virtual_texturing = Resources::lookup_shader("shaders/common/fullscreen_quad.vert");
  std::string f_shader_screen_space_virtual_texturing = Resources::lookup_shader("shaders/screen_space_virtual_texturing.frag");

  std::string v_shader_blit_vt_color = Resources::lookup_shader("shaders/common/fullscreen_quad.vert");
  std::string f_shader_blit_vt_color = Resources::lookup_shader("shaders/blit_virtual_texturing_colors.frag");
#endif
  screen_space_virtual_texturing_shader_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader_screen_space_virtual_texturing));
  screen_space_virtual_texturing_shader_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader_screen_space_virtual_texturing));

  blit_vt_color_to_gbuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader_blit_vt_color));
  blit_vt_color_to_gbuffer_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader_blit_vt_color));
  }

  ///////////////////////////////////////////////////////////////////////////////
  void DeferredVirtualTexturingRenderer::_create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims) {


    if(!fullscreen_quad_) {
      fullscreen_quad_.reset(new scm::gl::quad_geometry(ctx.render_device, 
                                                scm::math::vec2(-1.0f, -1.0f), scm::math::vec2(1.0f, 1.0f )));
    

      virtually_textured_color_attachment_ = ctx.render_device
        ->create_texture_2d(render_target_dims,
                            scm::gl::FORMAT_RGBA_32F,
                            1, 1, 1);

      screen_space_virtual_texturing_fbo_.reset();
      
      screen_space_virtual_texturing_fbo_ = ctx.render_device->create_frame_buffer();
      screen_space_virtual_texturing_fbo_->clear_attachments();

      screen_space_virtual_texturing_fbo_->attach_color_buffer(0,
                                                               virtually_textured_color_attachment_);

      nearest_sampler_state_ = ctx.render_device
        ->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

      linear_sampler_state_ = ctx.render_device
        ->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);

    }

    _create_physical_texture(ctx);
    _create_index_texture_hierarchy(ctx);

}


  /////////////////////////////////////////////////////////////////////////////////////////////
  void DeferredVirtualTexturingRenderer::set_global_substitution_map(SubstitutionMap const& smap) {}
  
  void DeferredVirtualTexturingRenderer::apply_cut_update(gua::RenderContext const& ctx, uint64_t cut_id, uint16_t ctx_id){

  }

  void DeferredVirtualTexturingRenderer::update_index_texture(gua::RenderContext const& ctx,uint64_t cut_id, uint32_t dataset_id, uint16_t context_id, const uint8_t *buf_cpu) {
    /*
    uint32_t size_index_texture = (uint32_t)vt::QuadTree::get_tiles_per_row((*vt::CutDatabase::get_instance().get_cut_map())[cut_id]->get_atlas()->getDepth() - 1);
    
    auto vector_of_vt_ptr = TextureDatabase::instance()->get_virtual_textures();

    for( auto const& vt_ptr : vector_of_vt_ptr ) {
      scm::math::vec3ui origin = scm::math::vec3ui(0, 0, 0);
      scm::math::vec3ui _index_texture_dimension = scm::math::vec3ui(size_index_texture, size_index_texture, 1);

      vt_ptr->update_sub_data(ctx, scm::gl::texture_region(origin, _index_texture_dimension), 0, scm::gl::FORMAT_RGBA_8UI, buf_cpu);
    }
    */

  }

  void DeferredVirtualTexturingRenderer::_init_vt(gua::RenderContext const& ctx, gua::node::SerializedCameraNode const& cam) {
    auto& vt_info_per_context = VirtualTexture2D::vt_info_per_context_;
    auto current_vt_info_per_context_iterator = vt_info_per_context.find(ctx.id);

    // if vt_info for this render context doesnt exist, register context
    if(vt_info_per_context.end() == current_vt_info_per_context_iterator ) { 
      auto& current_vt_info = vt_info_per_context[ctx.id];

      current_vt_info.context_id_ = ::vt::CutDatabase::get_instance().register_context();

      //std::cout << "Filled VT info for context: " << ctx.id << " with " << current_vt_info.context_id_ <<  "\n";
    }

    auto& current_vt_info = vt_info_per_context[ctx.id];
    auto camera_iter = current_vt_info.gua_camera_id_to_lamure_view_id_.find(cam.uuid);

    // if camera uuid is not in the list, register camera and create lamure view for it
    if(current_vt_info.gua_camera_id_to_lamure_view_id_.end() == camera_iter ) { 
      uint16_t lamure_view_id = ::vt::CutDatabase::get_instance().register_view();
      current_vt_info.gua_camera_id_to_lamure_view_id_[cam.uuid] = lamure_view_id;
      //std::cout << "Filled VT info for camera uuid: " << cam.uuid << " with " << lamure_view_id <<  "\n"; 
    }


  }

  void DeferredVirtualTexturingRenderer::_create_physical_texture(gua::RenderContext const& ctx) {
    auto& physical_texture_ptrs_per_context = VirtualTexture2D::physical_texture_ptr_per_context_;

    auto current_ctx_physical_texture_iterator = physical_texture_ptrs_per_context.find(ctx.id);

    if(physical_texture_ptrs_per_context.end() == current_ctx_physical_texture_iterator ) {
      auto& current_physical_texture_ptr = physical_texture_ptrs_per_context[ctx.id];

      current_physical_texture_ptr = std::make_shared<LayeredPhysicalTexture2D>();

      current_physical_texture_ptr->upload_to(ctx);
    }
  }

  void DeferredVirtualTexturingRenderer::_create_index_texture_hierarchy(gua::RenderContext const& ctx) {

    auto vector_of_vt_ptr = TextureDatabase::instance()->get_virtual_textures();


    for( auto const& vt_ptr : vector_of_vt_ptr ) {
      //scm::math::vec3ui origin = scm::math::vec3ui(0, 0, 0);
      //scm::math::vec3ui _index_texture_dimension = scm::math::vec3ui(size_index_texture, size_index_texture, 1);

      //std::cout << vt_ptr->uuid() << "\n";
      vt_ptr->upload_to(ctx);
      // get buf_cpu

      //vt_ptr->update_sub_data(ctx, scm::gl::texture_region(origin, _index_texture_dimension), 0, scm::gl::FORMAT_RGBA_8UI, buf_cpu);
    }
  }

  void DeferredVirtualTexturingRenderer::_register_cuts(gua::RenderContext const& ctx) {
    auto& vt_info_per_context = VirtualTexture2D::vt_info_per_context_;
    auto& current_vt_info = vt_info_per_context[ctx.id];
    uint16_t ctx_id = current_vt_info.context_id_;
    for ( auto const& view_entry : current_vt_info.gua_camera_id_to_lamure_view_id_){
      uint16_t view_id = view_entry.second;
      auto vector_of_vt_ptr = TextureDatabase::instance()->get_virtual_textures();
      for( auto const& vt_ptr : vector_of_vt_ptr ) {
        
        uint32_t tex_id = vt_ptr->get_lamure_texture_id();
        // render
        uint64_t cut_id = (((uint64_t) tex_id) << 32) | ((uint64_t) view_id << 16) | ((uint64_t) ctx_id);
        auto cut_iter = current_vt_info.cut_id_to_lamure_triple_.find(cut_id);

        // check if cut got registered already
        if (current_vt_info.cut_id_to_lamure_triple_.end() == cut_iter){
          current_vt_info.cut_id_to_lamure_triple_[cut_id] = 
              ::vt::CutDatabase::get_instance().register_cut(tex_id, view_id, ctx_id);

        }
      }
    }
  }

  void DeferredVirtualTexturingRenderer::_start_cut_update(gua::RenderContext const& ctx) {
    auto& vt_info_per_context = VirtualTexture2D::vt_info_per_context_;
    auto& current_vt_info = vt_info_per_context[ctx.id];

    if (!current_vt_info.cut_update_ && !current_vt_info.cut_updated_running_){
      current_vt_info.cut_update_ = &::vt::CutUpdate::get_instance();
      current_vt_info.cut_update_->start();
      current_vt_info.cut_updated_running_ = true;
    }
  }

  void DeferredVirtualTexturingRenderer::_apply_cut_update(gua::RenderContext const& ctx) {
    // auto *cut_db = &::vt::CutDatabase::get_instance();


    // for (::vt::cut_map_entry_type cut_entry : (*cut_db->get_cut_map())) {
    //     ::vt::Cut *cut = cut_db->start_reading_cut(cut_entry.first);

    //     if (!cut->is_drawn()) {
    //         cut_db->stop_reading_cut(cut_entry.first);
    //         continue;
    //     }

    //     std::set<uint16_t> updated_levels;

    //     for (auto position_slot_updated : cut->get_front()->get_mem_slots_updated()) {
    //         const ::vt::mem_slot_type *mem_slot_updated = cut_db->read_mem_slot_at(position_slot_updated.second);

    //         if (mem_slot_updated == nullptr || !mem_slot_updated->updated
    //             || !mem_slot_updated->locked || mem_slot_updated->pointer == nullptr) {
    //             if (mem_slot_updated == nullptr) {
    //                 std::cerr << "Mem slot at " << position_slot_updated.second << " is null" << std::endl;
    //             } else {
    //                 std::cerr << "Mem slot at " << position_slot_updated.second << std::endl;
    //                 std::cerr << "Mem slot #" << mem_slot_updated->position << std::endl;
    //                 std::cerr << "Tile id: " << mem_slot_updated->tile_id << std::endl;
    //                 std::cerr << "Locked: " << mem_slot_updated->locked << std::endl;
    //                 std::cerr << "Updated: " << mem_slot_updated->updated << std::endl;
    //                 std::cerr << "Pointer valid: " << (mem_slot_updated->pointer != nullptr) << std::endl;
    //             }

    //             throw std::runtime_error("updated mem slot inconsistency");
    //         }

    //         updated_levels.insert(vt::QuadTree::get_depth_of_node(mem_slot_updated->tile_id));

    //         // update_physical_texture_blockwise
    //         size_t slots_per_texture = ::vt::VTConfig::get_instance().get_phys_tex_tile_width() *
    //                                    ::vt::VTConfig::get_instance().get_phys_tex_tile_width();
    //         size_t layer = mem_slot_updated->position / slots_per_texture;
    //         size_t rel_slot_position = mem_slot_updated->position - layer * slots_per_texture;

    //         size_t x_tile = rel_slot_position % ::vt::VTConfig::get_instance().get_phys_tex_tile_width();
    //         size_t y_tile = rel_slot_position / ::vt::VTConfig::get_instance().get_phys_tex_tile_width();

    //         scm::math::vec3ui origin = scm::math::vec3ui(
    //                 (uint32_t) x_tile * ::vt::VTConfig::get_instance().get_size_tile(),
    //                 (uint32_t) y_tile * ::vt::VTConfig::get_instance().get_size_tile(), (uint32_t) layer);
    //         scm::math::vec3ui dimensions = scm::math::vec3ui(::vt::VTConfig::get_instance().get_size_tile(),
    //                                                          ::vt::VTConfig::get_instance().get_size_tile(), 1);


    //         auto physical_tex = VirtualTexture2D::physical_texture_ptr_per_context_[ctx.id]->get_physical_texture_ptr();

    //         ctx.render_context->update_sub_texture(physical_tex, scm::gl::texture_region(origin, dimensions), 0,
    //                                      scm::gl::FORMAT_RGB_8, mem_slot_updated->pointer);
    //     }


    //     for (auto position_slot_cleared : cut->get_front()->get_mem_slots_cleared()) {
    //         const ::vt::mem_slot_type *mem_slot_cleared = cut_db->read_mem_slot_at(position_slot_cleared.second);

    //         if (mem_slot_cleared == nullptr) {
    //             std::cerr << "Mem slot at " << position_slot_cleared.second << " is null" << std::endl;
    //         }

    //         updated_levels.insert(vt::QuadTree::get_depth_of_node(position_slot_cleared.first));
    //     }

    //     // update_index_texture
    //     for (uint16_t updated_level : updated_levels) {
    //         uint32_t size_index_texture = (uint32_t) ::vt::QuadTree::get_tiles_per_row(updated_level);

    //         scm::math::vec3ui origin = scm::math::vec3ui(0, 0, 0);
    //         scm::math::vec3ui dimensions = scm::math::vec3ui(size_index_texture, size_index_texture, 1);
    //         auto index_tex_hierarchy = 
    //         ctx.render_context->update_sub_texture(vt_.index_texture_hierarchy_.at(updated_level),
    //                                      scm::gl::texture_region(origin, dimensions), 0, scm::gl::FORMAT_RGBA_8UI,
    //                                      cut->get_front()->get_index(updated_level));

    //     }
    //     cut_db->stop_reading_cut(cut_entry.first);
    // }
    // ctx.render_context->sync();
  }

  void DeferredVirtualTexturingRenderer::update_physical_texture_blockwise(gua::RenderContext const& ctx, uint16_t context_id, const uint8_t *buf_texel, size_t slot_position) {
  }

  ///////////////////////////////////////////////////////////////////////////////


  void DeferredVirtualTexturingRenderer::_initialize_shader_programs(gua::RenderContext const& ctx) {
    {
      auto new_program = std::make_shared<ShaderProgram>();
      new_program->set_shaders(screen_space_virtual_texturing_shader_program_stages_);
      screen_space_virtual_texturing_shader_program_ = new_program;
    }
       
    {
      auto new_program = std::make_shared<ShaderProgram>();
      new_program->set_shaders(blit_vt_color_to_gbuffer_program_stages_);
      blit_vt_color_to_gbuffer_program_ = new_program;
    }
 
  }

  void DeferredVirtualTexturingRenderer::_check_shader_programs(gua::RenderContext const& ctx) {
    if(   (!screen_space_virtual_texturing_shader_program_) 
       || (!blit_vt_color_to_gbuffer_program_)) {
      _initialize_shader_programs(ctx);
    } 
  }


  void DeferredVirtualTexturingRenderer::collect_feedback(gua::RenderContext const& ctx) {
    auto& gua_layered_physical_texture_for_context = VirtualTexture2D::physical_texture_ptr_per_context_[ctx.id];

    auto feedback_lod_storage_ptr = gua_layered_physical_texture_for_context->get_feedback_lod_storage_ptr();
    auto feedback_lod_cpu_buffer_ptr = gua_layered_physical_texture_for_context->get_feedback_lod_cpu_buffer();

    std::size_t num_feedback_slots = gua_layered_physical_texture_for_context->get_num_feedback_slots();

    int32_t *feedback_lod = (int32_t *) ctx.render_context->map_buffer(feedback_lod_storage_ptr, scm::gl::ACCESS_READ_ONLY);

    memcpy(feedback_lod_cpu_buffer_ptr, feedback_lod, num_feedback_slots * size_of_format(scm::gl::FORMAT_R_32I));
    ctx.render_context->sync();

    ctx.render_context->unmap_buffer(feedback_lod_storage_ptr);
    ctx.render_context->clear_buffer_data(feedback_lod_storage_ptr, scm::gl::FORMAT_R_32I, nullptr);


    auto feedback_count_storage_ptr = gua_layered_physical_texture_for_context->get_feedback_count_storage_ptr();
    auto feedback_count_cpu_buffer_ptr = gua_layered_physical_texture_for_context->get_feedback_count_cpu_buffer();

    uint32_t *feedback_count = (uint32_t *) ctx.render_context->map_buffer(feedback_count_storage_ptr,
                                                                 scm::gl::ACCESS_READ_ONLY);
    memcpy(feedback_count_cpu_buffer_ptr, feedback_count, num_feedback_slots * size_of_format(scm::gl::FORMAT_R_32UI));
    ctx.render_context->sync();

    ctx.render_context->unmap_buffer(feedback_count_storage_ptr);
    ctx.render_context->clear_buffer_data(feedback_count_storage_ptr, scm::gl::FORMAT_R_32UI, nullptr);


    //for(int i = 0; i < num_feedback_slots; ++i) {
      //int32_t feedback_for_current_slot = feedback_lod_cpu_buffer_ptr[i];

      //if(0 != feedback_for_current_slot) {
        //std::cout << "LOD Feedback for slot " << i << " = " << feedback_for_current_slot << "\n";
      //}
    //}



    // give feedback after merge
    //vt_.cut_update_->feedback(vt_.feedback_lod_cpu_buffer_, vt_.feedback_count_cpu_buffer_);
  }


  ///////////////////////////////////////////////////////////////////////////////
  void DeferredVirtualTexturingRenderer::render(gua::Pipeline& pipe, PipelinePassDescription const& desc) {

    auto const& camera = pipe.current_viewstate().camera;
    scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();

    RenderContext const& ctx(pipe.get_context());

    /////////////////////// check and create gpu resources ///////////////////////

    _check_shader_programs(ctx);

    // fill VTInfo
    _init_vt(ctx, camera);

    // in here, index texture hierarchies and physical textures are created lazily
    _create_gpu_resources(ctx, render_target_dims);

    // register cut 
    _register_cuts(ctx);

    // start cut update
    _start_cut_update(ctx);

    /////////////////////// get data from lamure ///////////////////////////////
    ctx.render_context->sync();

    //apply cut update

    ctx.render_context->sync();
    /////////////////////////////render //////////////////////////////////////

    ctx.render_context
      ->clear_color_buffer(screen_space_virtual_texturing_fbo_, 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));


    // retrieve uv coordinates 
    {
      ctx.render_context->set_frame_buffer(screen_space_virtual_texturing_fbo_);
      screen_space_virtual_texturing_shader_program_->use(ctx);
      
      auto& gbuffer = *pipe.get_gbuffer();
      ctx.render_context->bind_texture(gbuffer.get_uv_buffer(), nearest_sampler_state_, 0);
      screen_space_virtual_texturing_shader_program_->apply_uniform(ctx, "gua_uv_buffer", 0);

      auto& current_physical_texture_ptr = VirtualTexture2D::physical_texture_ptr_per_context_[ctx.id];

      ctx.render_context->bind_texture(current_physical_texture_ptr->get_physical_texture_ptr(), linear_sampler_state_, 1);
      screen_space_virtual_texturing_shader_program_->apply_uniform(ctx, "layered_physical_texture", 1);



      auto vector_of_vt_ptr = TextureDatabase::instance()->get_virtual_textures();

      uint32_t global_texture_binding_idx = 2;

      std::string const hierarchical_index_texture_uniform_name = "hierarchical_idx_textures";
      for( auto const& vt_ptr : vector_of_vt_ptr ) {
        auto& current_index_texture_hierarchy = vt_ptr->get_index_texture_ptrs_for_context(ctx);

        for(auto const& index_texture_layer : current_index_texture_hierarchy) {
          ctx.render_context->bind_texture(index_texture_layer, linear_sampler_state_, global_texture_binding_idx);
          screen_space_virtual_texturing_shader_program_->apply_uniform(ctx, hierarchical_index_texture_uniform_name, 
                                                                        int32_t(global_texture_binding_idx), int32_t(global_texture_binding_idx-2) );
          //vt_ptr->upload_to(ctx);

          ++global_texture_binding_idx;
        }
      }


      ctx.render_context->apply();

      fullscreen_quad_->draw(ctx.render_context);

      screen_space_virtual_texturing_shader_program_->unuse(ctx);
    }



    auto& target = *pipe.current_viewstate().target;


    bool write_depth = false;
    target.bind(ctx, write_depth);
    target.set_viewport(ctx);
    //ctx.render_context->apply();

    blit_vt_color_to_gbuffer_program_->use(ctx);

    {

    ctx.render_context->bind_texture(virtually_textured_color_attachment_, nearest_sampler_state_, 0);
    blit_vt_color_to_gbuffer_program_->apply_uniform(ctx, "passed_vt_colors", 0);

    ctx.render_context->apply();
    fullscreen_quad_->draw(ctx.render_context);
    }

    blit_vt_color_to_gbuffer_program_->unuse(ctx);
    target.unbind(ctx);


    /////////////////////// send feedback data to lamure ///////////////////////////////
    ctx.render_context->sync();
    collect_feedback(ctx);
  }

}