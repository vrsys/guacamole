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
#include <gua/renderer/PipelinePass.hpp>

#include <gua/config.hpp>
#include <gua/renderer/ProgramFactory.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/LightTable.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

PipelinePass::PipelinePass(PipelinePassDescription const& d, RenderContext const& ctx)
  : shader_(std::make_shared<ShaderProgram>())
  , rasterizer_state_(nullptr)
  , depth_stencil_state_(nullptr)
  , blend_state_(nullptr)
  , needs_color_buffer_as_input_(d.needs_color_buffer_as_input_)
  , writes_only_color_buffer_(d.writes_only_color_buffer_)
  , doClear_(d.doClear_)
  , rendermode_(d.rendermode_)
  , process_(d.process_)
{
  if (!d.vertex_shader_.empty() && !d.fragment_shader_.empty()) {
    if (!d.geometry_shader_.empty()) {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
      ProgramFactory factory;
      shader_->create_from_sources(factory.read_from_file(d.vertex_shader_), 
                                   factory.read_from_file(d.geometry_shader_),  
                                   factory.read_from_file(d.fragment_shader_));
      //shader_->save_to_file("compiled_shaders", "shader");
#else
      shader_->create_from_sources(
          Resources::lookup_shader(d.vertex_shader_),
          Resources::lookup_shader(d.geometry_shader_),
          Resources::lookup_shader(d.fragment_shader_));
#endif
    } else {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
      ProgramFactory factory;
      shader_->create_from_sources(factory.read_from_file(d.vertex_shader_),
                                   factory.read_from_file(d.fragment_shader_));
      //shader_->save_to_file("compiled_shaders", "shader");
#else
      shader_->create_from_sources(
          Resources::lookup_shader(d.vertex_shader_),
          Resources::lookup_shader(d.fragment_shader_));
#endif
    }

    shader_->upload_to(ctx);
  }

  if (d.depth_stencil_state_) {
    depth_stencil_state_ =
        ctx.render_device->create_depth_stencil_state(*d.depth_stencil_state_);
  }
  if (d.blend_state_) {
    blend_state_ = ctx.render_device->create_blend_state(*d.blend_state_);
  }
  if (d.rasterizer_state_) {
    rasterizer_state_ =
      ctx.render_device->create_rasterizer_state(*d.rasterizer_state_);
  }
}

void PipelinePass::process(PipelinePassDescription const& desc, Pipeline& pipe) {

  auto const& ctx(pipe.get_context());

  if (desc.recompile_shaders_) {
    if (!desc.vertex_shader_.empty() && !desc.fragment_shader_.empty()) {
      if (!desc.geometry_shader_.empty()) {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
        ProgramFactory factory;
        shader_->create_from_sources(
          desc.vertex_shader_is_file_name_ ? factory.read_from_file(desc.vertex_shader_) : desc.vertex_shader_,
          desc.geometry_shader_is_file_name_ ? factory.read_from_file(desc.geometry_shader_) : desc.geometry_shader_,
          desc.fragment_shader_is_file_name_ ? factory.read_from_file(desc.fragment_shader_) : desc.fragment_shader_
          );
#else
        shader_->create_from_sources(
          desc.vertex_shader_is_file_name_ ? Resources::lookup_shader(desc.vertex_shader_) : desc.vertex_shader_,
          desc.geometry_shader_is_file_name_ ? Resources::lookup_shader(desc.geometry_shader_) : desc.geometry_shader_,
          desc.fragment_shader_is_file_name_ ? Resources::lookup_shader(desc.fragment_shader_) : desc.fragment_shader_
        );
#endif
      } else {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
        ProgramFactory factory;
        shader_->create_from_sources(
          desc.vertex_shader_is_file_name_ ? factory.read_from_file(desc.vertex_shader_) : desc.vertex_shader_,
          desc.fragment_shader_is_file_name_ ? factory.read_from_file(desc.fragment_shader_) : desc.fragment_shader_
          );
#else
        shader_->create_from_sources(
          desc.vertex_shader_is_file_name_ ? Resources::lookup_shader(desc.vertex_shader_) : desc.vertex_shader_,
          desc.fragment_shader_is_file_name_ ? Resources::lookup_shader(desc.fragment_shader_) : desc.fragment_shader_
          );
#endif
      }

      shader_->upload_to(ctx);
    }

    desc.recompile_shaders_ = false;
  }

  if (RenderMode::Custom == rendermode_) {
    process_(*this, desc, pipe);
  } else {
    pipe.get_gbuffer().bind(ctx, writes_only_color_buffer_);
    pipe.get_gbuffer().set_viewport(ctx);
    pipe.get_abuffer().bind(ctx);
    if (doClear_)
      pipe.get_gbuffer().clear_color(ctx);
    if (depth_stencil_state_)
      ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
    if (blend_state_)
      ctx.render_context->set_blend_state(blend_state_);
    if (rasterizer_state_)
      ctx.render_context->set_rasterizer_state(rasterizer_state_);
    shader_->use(ctx);

    for (auto const& u : desc.uniforms) {
      u.second.apply(ctx, u.first, ctx.render_context->current_program(), 0);
    }

    pipe.bind_gbuffer_input(shader_);
    pipe.bind_light_table(shader_);
    if (RenderMode::Callback == rendermode_) {
      process_(*this, desc, pipe);
    } else { // RenderMode::Quad
      pipe.draw_quad();
    }
    pipe.get_abuffer().unbind(ctx);
    pipe.get_gbuffer().unbind(ctx);
    ctx.render_context->reset_state_objects();
  }
}

}
