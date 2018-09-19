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

    }





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
    }*/

  }

  void DeferredVirtualTexturingRenderer::update_physical_texture_blockwise(gua::RenderContext const& ctx, uint16_t context_id, const uint8_t *buf_texel, size_t slot_position) {

  /*
    auto phy_tex_ptr = ctx.physical_texture;

    if(phy_tex_ptr == nullptr) {
      std::cout << "physical_context is nullptr\n";
    } 
    else 
    {
      size_t slots_per_texture = vt::VTConfig::get_instance().get_phys_tex_tile_width() * vt::VTConfig::get_instance().get_phys_tex_tile_width();
      size_t layer = slot_position / slots_per_texture;
      size_t rel_slot_position = slot_position - layer * slots_per_texture;
      size_t x_tile = rel_slot_position % vt::VTConfig::get_instance().get_phys_tex_tile_width();
      size_t y_tile = rel_slot_position / vt::VTConfig::get_instance().get_phys_tex_tile_width();

      scm::math::vec3ui origin = scm::math::vec3ui((uint32_t)x_tile * vt::VTConfig::get_instance().get_size_tile(), (uint32_t)y_tile * vt::VTConfig::get_instance().get_size_tile(), (uint32_t)layer);
      scm::math::vec3ui dimensions = scm::math::vec3ui(vt::VTConfig::get_instance().get_size_tile(), vt::VTConfig::get_instance().get_size_tile(), 1);

      ctx.render_context->update_sub_texture(phy_tex_ptr, scm::gl::texture_region(origin, dimensions), 0, PhysicalTexture2D::get_tex_format(), buf_texel);
    }
    */
  }

  ///////////////////////////////////////////////////////////////////////////////
  void DeferredVirtualTexturingRenderer::collect_feedback(gua::RenderContext const& ctx){

    /*
    using namespace scm::math;
    using namespace scm::gl;

    uint32_t *feedback = (uint32_t *)ctx.render_context->map_buffer(ctx.feedback_storage, ACCESS_READ_ONLY);

    memcpy(ctx.feedback_cpu_buffer, feedback, ctx.size_feedback * size_of_format(FORMAT_R_32UI));

    ctx.render_context->sync();

    auto *_cut_update = &vt::CutUpdate::get_instance();
    _cut_update->feedback(ctx.feedback_cpu_buffer);

    ctx.render_context->unmap_buffer(ctx.feedback_storage);
    ctx.render_context->clear_buffer_data(ctx.feedback_storage, FORMAT_R_32UI, nullptr);
    */
  }


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

  void DeferredVirtualTexturingRenderer::_update_index_texture_hierarchies(gua::RenderContext const& ctx) {

    auto vector_of_vt_ptr = TextureDatabase::instance()->get_virtual_textures();


    for( auto const& vt_ptr : vector_of_vt_ptr ) {
      //scm::math::vec3ui origin = scm::math::vec3ui(0, 0, 0);
      //scm::math::vec3ui _index_texture_dimension = scm::math::vec3ui(size_index_texture, size_index_texture, 1);

      std::cout << vt_ptr->uuid() << "\n";
      // get buf_cpu

      //vt_ptr->update_sub_data(ctx, scm::gl::texture_region(origin, _index_texture_dimension), 0, scm::gl::FORMAT_RGBA_8UI, buf_cpu);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  void DeferredVirtualTexturingRenderer::render(gua::Pipeline& pipe, PipelinePassDescription const& desc) {


    RenderContext const& ctx(pipe.get_context());

    // update index texture hierarchies (~ pro texture atlas)
    _update_index_texture_hierarchies(ctx);

    // update physical texture




    auto const& camera = pipe.current_viewstate().camera;
    scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();


    _check_shader_programs(ctx);
    _create_gpu_resources(ctx, render_target_dims);


    ctx.render_context
      ->clear_color_buffer(screen_space_virtual_texturing_fbo_, 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));


    // retrieve uv coordinates 
    {
      ctx.render_context->set_frame_buffer(screen_space_virtual_texturing_fbo_);
      screen_space_virtual_texturing_shader_program_->use(ctx);
      ctx.render_context->apply();

      fullscreen_quad_->draw(ctx.render_context);

      screen_space_virtual_texturing_shader_program_->unuse(ctx);
    }



    auto& target = *pipe.current_viewstate().target;

    bool write_depth = true;
    target.bind(ctx, write_depth);
    target.set_viewport(ctx);
    ctx.render_context->apply();



    blit_vt_color_to_gbuffer_program_->use(ctx);

    {
    ctx.render_context->apply();
    fullscreen_quad_->draw(ctx.render_context);
    }

    blit_vt_color_to_gbuffer_program_->unuse(ctx);
    target.unbind(ctx);


  }

}