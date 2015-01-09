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
#include <gua/video3d/Video3DRenderer.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/memory.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/MaterialShader.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/FrameBufferObject.hpp>
#include <gua/renderer/WindowBase.hpp>
#include <gua/video3d/Video3DResource.hpp>
#include <gua/video3d/Video3DNode.hpp>

#include <scm/gl_core/render_device/context_guards.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Video3DRenderer::Video3DRenderer()
  : initialized_(false)
{
  ResourceFactory factory;

  // create depth shader
  std::vector<ShaderProgramStage> warp_pass_stages;
  warp_pass_stages.push_back( ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,
                                                 factory.read_shader_file("resources/warp_pass.vert")));
  warp_pass_stages.push_back( ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER,
                                                 factory.read_shader_file("resources/warp_pass.geom")));
  warp_pass_stages.push_back( ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER,
                                                 factory.read_shader_file("resources/warp_pass.frag")));

  warp_pass_program_ = std::make_shared<ShaderProgram>();
  warp_pass_program_->set_shaders(warp_pass_stages);

  // create final shader description
  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,
                                               factory.read_shader_file("resources/blend_pass.vert")));
  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER,
                                               factory.read_shader_file("resources/blend_pass.frag")));
}

////////////////////////////////////////////////////////////////////////////////

void Video3DRenderer::render(Pipeline& pipe)
{
  auto const& ctx(pipe.get_context());
  
  if (!initialized_) {
    initialized_ = true;

    warp_pass_program_->upload_to(ctx);

    // TODO:::::
    // if (warp_color_result_) {
    //   return upload_succeeded;
    // }
    // else {
    //     // continue instantiation below
    // }

    // initialize Texture Arrays (kinect depths & colors)
    warp_color_result_ = ctx.render_device->create_texture_2d(
                  pipe.get_camera().config.resolution(),
                  scm::gl::FORMAT_RGBA_32F,
                  1,
                  MAX_NUM_KINECTS,
                  1
                  );

    warp_depth_result_ = ctx.render_device->create_texture_2d(
                  pipe.get_camera().config.resolution(),
                  scm::gl::FORMAT_D32F,
                  1,
                  MAX_NUM_KINECTS,
                  1
                  );

    warp_result_fbo_ = ctx.render_device->create_frame_buffer();
    linear_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
    nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
    depth_stencil_state_warp_pass_ = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
    depth_stencil_state_blend_pass_ = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
    no_bfc_rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);
  }


  auto objects(pipe.get_scene().nodes.find(std::type_index(typeid(node::Video3DNode))));
  int view_id(pipe.get_camera().config.get_view_id());

  if (objects != pipe.get_scene().nodes.end() && objects->second.size() > 0) {

    for (auto& o: objects->second) {

      auto video_node(reinterpret_cast<node::Video3DNode*>(o));
      auto video_name(video_node->get_video_name());

      if (!GeometryDatabase::instance()->contains(video_name)) {
        gua::Logger::LOG_WARNING << "Video3DRenderer::draw(): No such video." << video_name << ", " << std::endl;
        continue;
      }

      auto video3d_ressource = std::static_pointer_cast<Video3DResource>(GeometryDatabase::instance()->lookup(video_name));
      if (!video3d_ressource) {
        gua::Logger::LOG_WARNING << "Video3DRenderer::draw(): Invalid video." << std::endl;
        continue;
      }

      // update stream data
      video3d_ressource->update_buffers(ctx);

      UniformValue model_matrix(video_node->get_cached_world_transform());
      UniformValue normal_matrix(scm::math::transpose(scm::math::inverse(video_node->get_cached_world_transform())));

      {
        // single texture only
        scm::gl::context_all_guard guard(ctx.render_context);

        ctx.render_context->set_rasterizer_state(no_bfc_rasterizer_state_);
        ctx.render_context->set_depth_stencil_state(depth_stencil_state_warp_pass_);

        // set uniforms
        ctx.render_context->bind_texture(video3d_ressource->depth_array(ctx), nearest_sampler_state_, 0);
        warp_pass_program_->get_program(ctx)->uniform_sampler("depth_video3d_texture", 0);

        warp_pass_program_->apply_uniform(ctx, "gua_normal_matrix", normal_matrix);
        warp_pass_program_->apply_uniform(ctx, "gua_model_matrix", model_matrix);
        warp_pass_program_->set_uniform(ctx, int(1), "bbxclip");

        auto bbox(video3d_ressource->get_bounding_box());
        warp_pass_program_->set_uniform(ctx, bbox.min, "bbx_min");
        warp_pass_program_->set_uniform(ctx, bbox.max, "bbx_max");

        // pre passes
        for (unsigned layer = 0; layer != video3d_ressource->number_of_cameras(); ++layer)
        {
          // configure fbo
          warp_result_fbo_->clear_attachments();
          warp_result_fbo_->attach_depth_stencil_buffer(warp_depth_result_, 0, layer);
          warp_result_fbo_->attach_color_buffer(0, warp_color_result_, 0, layer);

          // bind and clear fbo
          ctx.render_context->set_frame_buffer(warp_result_fbo_);
          ctx.render_context->clear_depth_stencil_buffer(warp_result_fbo_);
          ctx.render_context->clear_color_buffer(warp_result_fbo_, 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));
          ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2ui(0,0), warp_color_result_->dimensions()));

          // set uniforms
          warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).getTexSizeInvD(), "tex_size_inv");
          warp_pass_program_->set_uniform(ctx, int(layer), "layer");


          if (video3d_ressource)
          {
            warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).getImageDToEyeD(), "image_d_to_eye_d");
            warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).getEyeDToWorld(), "eye_d_to_world");
            warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).getEyeDToEyeRGB(), "eye_d_to_eye_rgb");
            warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).getEyeRGBToImageRGB(), "eye_rgb_to_image_rgb");

            ctx.render_context->bind_texture(video3d_ressource->cv_xyz(ctx,layer), linear_sampler_state_, 1);
            warp_pass_program_->get_program(ctx)->uniform_sampler("cv_xyz", 1);

            ctx.render_context->bind_texture(video3d_ressource->cv_uv(ctx,layer), linear_sampler_state_, 2);
            warp_pass_program_->get_program(ctx)->uniform_sampler("cv_uv", 2);

            warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).cv_min_d, "cv_min_d");
            warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).cv_max_d, "cv_max_d");

            warp_pass_program_->use(ctx);
            {
              // ctx.render_context->apply();
              video3d_ressource->draw(ctx);
            }
            warp_pass_program_->unuse(ctx);
          }

          ctx.render_context->reset_framebuffer();
        }
      }

      // get material dependent shader
      std::shared_ptr<ShaderProgram> current_shader;

      MaterialShader* current_material = video_node->get_material()->get_shader();
      if (current_material) {

        auto shader_iterator = programs_.find(current_material);
        if (shader_iterator != programs_.end())
        {
          current_shader = shader_iterator->second;
        }
        else {
          current_shader = std::make_shared<ShaderProgram>();
          current_shader->set_shaders(program_stages_, std::list<std::string>(), false,
                                      current_material->generate_substitution_map());
          programs_[current_material] = current_shader;
        }           
      }
      else {
        Logger::LOG_WARNING << "Video3DPass::render(): Cannot find material: " << video_node->get_material()->get_shader_name() << std::endl;
      }

      current_shader->use(ctx);

      pipe.get_gbuffer().bind(ctx, false);
      pipe.get_gbuffer().set_viewport(ctx);

      {
        // single texture only
        scm::gl::context_all_guard guard(ctx.render_context);

        ctx.render_context->set_depth_stencil_state(depth_stencil_state_warp_pass_);

        // second pass
        {
          if (video3d_ressource)
          {
            current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_matrix);
            current_shader->apply_uniform(ctx, "gua_model_matrix", model_matrix);

            // needs to be multiplied with scene scaling
            current_shader->set_uniform(ctx, 0.075f, "epsilon");
            current_shader->set_uniform(ctx, int(video3d_ressource->number_of_cameras()), "numlayers");
            current_shader->set_uniform(ctx, int(video3d_ressource->do_overwrite_normal()), "overwrite_normal");
            current_shader->set_uniform(ctx, video3d_ressource->get_overwrite_normal(), "o_normal");

            ctx.render_context->bind_texture(warp_color_result_, nearest_sampler_state_, 0);
            current_shader->get_program(ctx)->uniform_sampler("quality_texture", 0);

            ctx.render_context->bind_texture(warp_depth_result_, nearest_sampler_state_, 1);
            current_shader->get_program(ctx)->uniform_sampler("depth_texture", 1);

            ctx.render_context->bind_texture(video3d_ressource->color_array(ctx), linear_sampler_state_, 2);
            current_shader->get_program(ctx)->uniform_sampler("video_color_texture", 2);
            
            video_node->get_material()->apply_uniforms(ctx, current_shader.get(), view_id);

            ctx.render_context->apply();
            pipe.draw_quad();
          }
        }
        current_shader->unuse(ctx);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

}
