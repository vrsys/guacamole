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

#include <gua/renderer/LinkedListAccumSubRenderer.hpp>
#include <lamure/ren/controller.h>

namespace gua {

  LinkedListAccumSubRenderer::LinkedListAccumSubRenderer() : PLodSubRenderer() {
  	_load_shaders();
  }

  void LinkedListAccumSubRenderer::create_gpu_resources(gua::RenderContext const& ctx,
                                              scm::math::vec2ui const& render_target_dims,
                                              gua::plod_shared_resources& shared_resources) {

  	// attachments

    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_FRAG_COUNT] = ctx.render_device
      ->create_texture_2d(render_target_dims, 
                          scm::gl::FORMAT_R_32UI, 
                          1, 1, 1);

    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_MIN_ES_DIST] = ctx.render_device
      ->create_texture_2d(render_target_dims, 
                          scm::gl::FORMAT_R_32UI, 
                          1, 1, 1);

    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_DUMMY_ATTACHMENT] = ctx.render_device->
      create_texture_2d(render_target_dims,
                        scm::gl::FORMAT_R_8UI,
                        1, 1, 1);



    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
    lamure::ren::policy* policy = lamure::ren::policy::get_instance();


    size_t max_num_surfels = std::floor( policy->render_budget_in_mb() * 1024 * 1024 
                             / database->get_primitive_size(lamure::ren::bvh::primitive_type::POINTCLOUD) );

    size_t const MAX_ATTRIBUTE_TEX_WIDTH = 8192;

    size_t x_res_attrib = MAX_ATTRIBUTE_TEX_WIDTH;
    size_t y_res_attrib = std::ceil(max_num_surfels / float(x_res_attrib));

    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_PBR_IMAGE] =  ctx.render_device->
      create_texture_2d(scm::math::vec2ui(x_res_attrib, y_res_attrib), scm::gl::FORMAT_RGBA_8 , 1, 1, 1);
    resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_NORMAL_IMAGE] =  ctx.render_device->
      create_texture_2d(scm::math::vec2ui(x_res_attrib, y_res_attrib), scm::gl::FORMAT_RGBA_8 , 1, 1, 1);

    size_t const NUM_BLENDED_FRAGS = 20;
    size_t const NUM_16BIT_LINKED_LIST_COMPONENTS = 4;
    size_t const total_num_pixels = render_target_dims[0] * render_target_dims[1] * NUM_BLENDED_FRAGS;


    resource_ptrs_.tex_buffers_[plod_shared_resources::TextureBufferID::LINKED_LIST_BUFFER] = ctx.render_device
      ->create_texture_buffer(scm::gl::FORMAT_RGBA_16UI, 
                              scm::gl::USAGE_DYNAMIC_COPY, 
                              sizeof(uint16_t) * NUM_16BIT_LINKED_LIST_COMPONENTS * total_num_pixels);


    ///////////////////////////////////////////
    _register_shared_resources(shared_resources);
    ///////////////////////////////////////////

    // fbo
    custom_FBO_ptr_ = ctx.render_device->create_frame_buffer();
    //custom_FBO_ptr_->clear_attachments();
    //custom_FBO_ptr_->attach_depth_stencil_buffer(shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH]);
    //custom_FBO_ptr_->attach_color_buffer(0, shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_FRAG_COUNT]);
    custom_FBO_ptr_->attach_color_buffer(0, shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_DUMMY_ATTACHMENT]);
    //state objects
    //custom_FBO_ptr_->attach_color_buffer(1, shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH]);


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


  }

  void LinkedListAccumSubRenderer::
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


  auto frag_count_image_ptr = shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_FRAG_COUNT];
  //auto dist_image_ptr = shared_resources.attachments_[plod_shared_resources::AttachmentID::DEPTH_PASS_LIN_DEPTH];
  auto dist_image_ptr = shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_MIN_ES_DIST];

  ctx.render_context
    ->clear_image_data(frag_count_image_ptr,
    0,
    frag_count_image_ptr->format(),
    0
  );


  ctx.render_context
    ->clear_image_data(dist_image_ptr,
    0,
    scm::gl::FORMAT_R_32UI,
    0
  );


  ctx.render_context->apply();


  MaterialShader* current_material(nullptr);
  std::shared_ptr<ShaderProgram> current_material_program;



    //std::string const gpu_query_name_accum_pass = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLodRenderer::AccumulationPass";
    //pipe.begin_gpu_query(ctx, gpu_query_name_accum_pass);

    //set accumulation pass states
    ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);

    ctx.render_context->set_frame_buffer(custom_FBO_ptr_);


    auto pbr_image_ptr    = shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_PBR_IMAGE];
    auto normal_image_ptr = shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_NORMAL_IMAGE];

    auto linked_list_buffer_tex_ptr = shared_resources.tex_buffers_[plod_shared_resources::TextureBufferID::LINKED_LIST_BUFFER];


    int view_id(camera.config.get_view_id());

    bool program_changed = false;
    //loop through all models and render accumulation pass
    for (auto const& object : sorted_models) {

      auto plod_node(reinterpret_cast<node::PLodNode*>(object));
      lamure::model_t model_id = controller->deduce_model_id(plod_node->get_geometry_description());


      current_material = plod_node->get_material()->get_shader();


      current_material_program = _get_material_dependent_shader_program(current_material,
                                                                        current_material_program,
                                                                        program_changed);
      auto const& camera = pipe.current_viewstate().camera;

      scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();

      using namespace lamure::ren;
      using namespace scm::gl;
      using namespace scm::math;

      model_database* database = model_database::get_instance();

      uint32_t number_of_surfels_per_node = database->get_primitives_per_node();




      //bind attribute images
      ctx.render_context->bind_image(pbr_image_ptr, pbr_image_ptr->format(), scm::gl::access_mode::ACCESS_WRITE_ONLY, 0);
      ctx.render_context->bind_image(normal_image_ptr, normal_image_ptr->format(), scm::gl::access_mode::ACCESS_WRITE_ONLY, 1);

      //bind linked list images
      ctx.render_context->bind_image(linked_list_buffer_tex_ptr, scm::gl::FORMAT_RGBA_16UI, scm::gl::access_mode::ACCESS_READ_WRITE, 2);
      ctx.render_context->bind_image(frag_count_image_ptr, frag_count_image_ptr->format(), scm::gl::access_mode::ACCESS_READ_WRITE, 3);
      ctx.render_context->bind_image(dist_image_ptr, scm::gl::FORMAT_R_32UI, scm::gl::access_mode::ACCESS_READ_WRITE, 4);
      //ctx.render_context->bind_image(min_es_dist_image_ptr, min_es_dist_image_ptr->format(), scm::gl::access_mode::ACCESS_READ_WRITE,3);
      ctx.render_context->apply();

      auto plod_resource = plod_node->get_geometry();

      //retrieve frustum culling results      
      std::unordered_set<lamure::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[plod_node];

      if (plod_resource && current_material_program) {

        if (program_changed) {
          current_material_program->unuse(ctx);
          current_material_program->use(ctx);
        }

        _upload_model_dependent_uniforms(current_material_program, ctx, plod_node, pipe);



      plod_node->get_material()->apply_uniforms(ctx, current_material_program.get(), view_id);

      current_material_program->apply_uniform(ctx, "point_size_factor", float(1.0) );

      uint32_t num_blend_f = 20;

      current_material_program->apply_uniform(ctx, "num_blended_frags", int(num_blend_f) );

      ctx.render_context->apply();

      plod_resource->draw(ctx,
                          context_id,
                          lamure_view_id,
                          model_id,
                          controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, ctx.render_device),
                          nodes_in_frustum,
                          scm::gl::primitive_topology::PRIMITIVE_POINT_LIST,
                          pipe.current_viewstate().frustum.get_view() * plod_node->get_cached_world_transform(),
                          true);

        program_changed = false;
      }
      else {
        Logger::LOG_WARNING << "PLodRenderer::render(): Cannot find ressources for node: " << plod_node->get_name() << std::endl;
      }
    }

    current_material_program->unuse(ctx);

    //pipe.end_gpu_query(ctx, gpu_query_name_accum_pass);

  }

  void LinkedListAccumSubRenderer::_load_shaders() {
    //create stages only with one thread!
    if(!shaders_loaded_) {

#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
    ResourceFactory factory;
    shader_stages_.clear();
    shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/plod/linked_list_splatting/p01_ll_accum.vert")));
    shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/plod/linked_list_splatting/p01_ll_accum.geom")));
    shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/plod/linked_list_splatting/p01_ll_accum.frag")));
    shaders_loaded_ = true;
   }
  }

  void LinkedListAccumSubRenderer::_upload_model_dependent_uniforms(std::shared_ptr<ShaderProgram> current_material_shader, RenderContext const& ctx, node::PLodNode* plod_node, gua::Pipeline& pipe) {
    auto const& frustum = pipe.current_viewstate().frustum;

    auto const& scm_model_matrix = plod_node->get_cached_world_transform();
    auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
    auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
    auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));
    auto scm_inv_trans_model_view_matrix = scm::math::transpose(scm::math::inverse(scm_model_view_matrix));

    current_material_shader->apply_uniform(ctx, "gua_model_matrix", math::mat4f(scm_model_matrix));
    current_material_shader->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(scm_model_view_matrix));
    current_material_shader->apply_uniform(ctx, "gua_model_view_projection_matrix", math::mat4f(scm_model_view_projection_matrix));
    current_material_shader->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(scm_normal_matrix));
    
    current_material_shader->apply_uniform(ctx, "enable_backface_culling", plod_node->get_enable_backface_culling_by_normal());
    current_material_shader->apply_uniform(ctx, "max_surfel_radius", plod_node->get_max_surfel_radius());
    current_material_shader->apply_uniform(ctx, "radius_scaling", plod_node->get_radius_scale());
    current_material_shader->apply_uniform(ctx, "inverse_transpose_model_view_matrix",  math::mat4f(scm_inv_trans_model_view_matrix) );
  }

} //namespace gua