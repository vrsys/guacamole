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

#include <gua/renderer/HoleFillingSubRenderer.hpp>
#include <lamure/ren/controller.h>

namespace gua {

  HoleFillingSubRenderer::HoleFillingSubRenderer() : PLodSubRenderer(), fullscreen_quad_(nullptr) {
  	_load_shaders();
  }

  void HoleFillingSubRenderer::create_gpu_resources(gua::RenderContext const& ctx,
                                                      scm::math::vec2ui const& render_target_dims,
                                                      gua::plod_shared_resources& shared_resources) {

    nearest_sampler_state_ = ctx.render_device
      ->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
  }

  void HoleFillingSubRenderer::
  render_sub_pass(Pipeline& pipe, PipelinePassDescription const& desc,
  				  gua::plod_shared_resources& shared_resources,
  				  std::vector<node::Node*>& sorted_models,
                  std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t> >& nodes_in_frustum_per_model,
                  lamure::context_t context_id,
  				  lamure::view_t lamure_view_id
  				  ) {


  	RenderContext const& ctx(pipe.get_context());
    auto& target = *pipe.current_viewstate().target;

	  _check_for_shader_program();

	  assert(shader_program_);



    if(!fullscreen_quad_) {
      fullscreen_quad_.reset(new scm::gl::quad_geometry(ctx.render_device, 
                                                scm::math::vec2(-1.0f, -1.0f), scm::math::vec2(1.0f, 1.0f )));
    }
    bool write_depth = true;
    target.bind(ctx, write_depth);

    scm::gl::context_all_guard context_guard(ctx.render_context);

    shader_program_->use(ctx);
    {

      _upload_hole_filling_pass_uniforms(ctx, shared_resources);

      ctx.render_context->apply();

      fullscreen_quad_->draw(ctx.render_context);
    }
    shader_program_->unuse(ctx);

    target.unbind(ctx);

  }

  void HoleFillingSubRenderer::_load_shaders() {
    //create stages only with one thread!
    if(!shaders_loaded_) {

#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
    ResourceFactory factory;

    shader_stages_.clear();
    shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, 
                             factory.read_shader_file("resources/shaders/plod/linked_list_splatting/p03_fill_holes.vert")) );
    shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, 
                             factory.read_shader_file("resources/shaders/plod/linked_list_splatting/p03_fill_holes.frag")) );

    shaders_loaded_ = true;
   }
  }

  void HoleFillingSubRenderer::_upload_hole_filling_pass_uniforms(RenderContext const& ctx, 
                                                                     gua::plod_shared_resources& shared_resources) {

    ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_RESOLVE_PASS_COLOR_IMAGE], nearest_sampler_state_, 0);
    ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_RESOLVE_PASS_NORMAL_IMAGE], nearest_sampler_state_, 1);
    ctx.render_context->bind_texture(shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_RESOLVE_PASS_PBR_IMAGE], nearest_sampler_state_, 2);
    ctx.render_context->bind_image(shared_resources.attachments_[plod_shared_resources::AttachmentID::LINKED_LIST_ACCUM_PASS_MIN_ES_DIST], scm::gl::FORMAT_R_32UI, scm::gl::access_mode::ACCESS_READ_ONLY, 3);

  }


} //namespace gua