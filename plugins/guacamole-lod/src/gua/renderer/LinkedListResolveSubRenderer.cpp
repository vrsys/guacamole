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

#include <gua/renderer/LodResource.hpp>

#include <gua/renderer/LinkedListResolveSubRenderer.hpp>
#include <lamure/ren/controller.h>

namespace gua {

  LinkedListResolveSubRenderer::LinkedListResolveSubRenderer() : PLodSubRenderer(), fullscreen_quad_(nullptr){
  	_load_shaders();
  }

  void LinkedListResolveSubRenderer::create_gpu_resources(gua::RenderContext const& ctx,
                                              scm::math::vec2ui const& render_target_dims,
                                              gua::plod_shared_resources& shared_resources) {


    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_RESOLVE_PASS_COLOR_IMAGE] = ctx.render_device
      ->create_texture_2d(render_target_dims, 
                          scm::gl::FORMAT_RGBA_8, 
                          1, 1, 1);

    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_RESOLVE_PASS_NORMAL_IMAGE] = ctx.render_device
      ->create_texture_2d(render_target_dims, 
                          scm::gl::FORMAT_RGBA_8, 
                          1, 1, 1);

  
    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_RESOLVE_PASS_PBR_IMAGE] = ctx.render_device
      ->create_texture_2d(render_target_dims, 
                          scm::gl::FORMAT_RGBA_8, 
                          1, 1, 1);
  

    _register_shared_resources(shared_resources);

  }

  void LinkedListResolveSubRenderer::
  bind_storage_buffer(scm::gl::buffer_ptr buffer, RenderContext const& ctx) {

      lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
      lamure::ren::policy* policy = lamure::ren::policy::get_instance();

      size_t size_of_node_in_bytes = database->get_primitive_size(lamure::ren::bvh::primitive_type::POINTCLOUD) * database->get_primitives_per_node();
      size_t render_budget_in_mb = policy->render_budget_in_mb();

      size_t num_slots = (render_budget_in_mb * 1024u * 1024u) / size_of_node_in_bytes;

      //shader_program_->storage_buffer("point_attrib_ssbo", 0);
      //ctx.render_context->bind_storage_buffer(buffer, 0, 0, num_slots * size_of_node_in_bytes);
      ctx.render_context->bind_storage_buffer(buffer, 2);
  }

  void LinkedListResolveSubRenderer::
  render_sub_pass(Pipeline& pipe, PipelinePassDescription const& desc,
  				        gua::plod_shared_resources& shared_resources,
  				        std::vector<node::Node*>& sorted_models,
                  std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t> >& nodes_in_frustum_per_model,
                  lamure::context_t context_id,
  				        lamure::view_t lamure_view_id
  				  ) {

  auto const& camera = pipe.current_viewstate().camera;
  RenderContext const& ctx(pipe.get_context());
  lamure::ren::controller* controller = lamure::ren::controller::get_instance(); 

  scm::gl::context_all_guard context_guard(ctx.render_context);

    if(!custom_FBO_ptr_) {
      // fbo
      custom_FBO_ptr_ = ctx.render_device->create_frame_buffer();
      custom_FBO_ptr_->clear_attachments();
      //custom_FBO_ptr_->attach_depth_stencil_buffer(shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH]);

      //state objects


      depth_state_disable_ = ctx.render_device
        ->create_depth_stencil_state(false, 
                                     true, 
                                     scm::gl::COMPARISON_NEVER);

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


      filter_nearest_ = ctx.render_device->
        create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);

    }


    if(!fullscreen_quad_) {
      fullscreen_quad_.reset(new scm::gl::quad_geometry(ctx.render_device, 
                                                scm::math::vec2(-1.0f, -1.0f), scm::math::vec2(1.0f, 1.0f )));
    }

  {



    //set accumulation pass states
    ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);


    //custom_FBO_ptr_->attach_depth_stencil_buffer(shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH]);
    custom_FBO_ptr_->attach_color_buffer(0, resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_RESOLVE_PASS_COLOR_IMAGE]);
    custom_FBO_ptr_->attach_color_buffer(1, resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_RESOLVE_PASS_NORMAL_IMAGE]);
    custom_FBO_ptr_->attach_color_buffer(2, resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_RESOLVE_PASS_PBR_IMAGE]);
    ctx.render_context->set_frame_buffer(custom_FBO_ptr_);


    int view_id(camera.config.get_view_id());

    _check_for_shader_program();

    assert(shader_program_);

    shader_program_->use(ctx);

    auto frag_count_image_ptr = shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_FRAG_COUNT];
    auto min_es_dist_image_ptr = shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_MIN_ES_DIST];
    //auto min_es_dist_image_ptr = shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH];

    auto texture_buffer_ptr = shared_resources.tex_buffers_[plod_shared_resources::TextureBufferID::LINKED_LIST_BUFFER];

    ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_PBR_IMAGE], filter_nearest_, 0);
    ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_NORMAL_IMAGE], filter_nearest_, 1);

    ctx.render_context->bind_image(texture_buffer_ptr,
    scm::gl::FORMAT_RGBA_16UI, scm::gl::access_mode::ACCESS_READ_ONLY, 2);
    ctx.render_context->bind_image(frag_count_image_ptr,
    frag_count_image_ptr->format(), scm::gl::access_mode::ACCESS_READ_ONLY,3);
    ctx.render_context->bind_image(min_es_dist_image_ptr,
    scm::gl::FORMAT_R_32UI, scm::gl::access_mode::ACCESS_READ_ONLY,4);

    auto const surfel_buffer = controller->get_context_buffer(context_id, ctx.render_device);
    bind_storage_buffer(surfel_buffer, ctx);

    //ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_FRAG_COUNT], filter_nearest_, 1);

    auto const& camera = pipe.current_viewstate().camera;

    scm::math::vec2ui const& render_target_dims = camera.config.get_resolution(); 

    shader_program_->apply_uniform(ctx, "point_size_factor", float(1.0) );

    uint32_t num_blend_f = 20;
    shader_program_->apply_uniform(ctx, "num_blended_frags", int(num_blend_f) );

    shader_program_->apply_uniform(ctx, "in_surfels_pbr", 0 );
    shader_program_->apply_uniform(ctx, "in_surfels_normal", 1);
    shader_program_->apply_uniform(ctx, "linked_list_buffer", 2 );
    shader_program_->apply_uniform(ctx, "fragment_count_img", 3 );
    shader_program_->apply_uniform(ctx, "min_es_distance_image", 4 );

    ctx.render_context->apply();
    //loop through all models and render accumulation pass


    if (shader_program_) {
      fullscreen_quad_->draw(ctx.render_context);
    }
    else {
      Logger::LOG_WARNING << "PLodRenderer::render(): Cannot use shader program. " << std::endl;
    }
    
  
  }
  
  shader_program_->unuse(ctx);
  
  }

  void LinkedListResolveSubRenderer::_load_shaders() {
    //create stages only with one thread!
    if(!shaders_loaded_) {

#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
    ResourceFactory factory;
    shader_stages_.clear();
    shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/gbuffer/lod/linked_list_splatting/p02_ll_resolve.vert")));
    shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/gbuffer/lod/linked_list_splatting/p02_ll_resolve.frag")));
    shaders_loaded_ = true;
   }
  }

} //namespace gua