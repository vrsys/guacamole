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
#include <gua/renderer/CompositePass.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/LayerMapping.hpp>
#include <gua/databases.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

CompositePass::CompositePass(Pipeline* pipeline) :
	GeometryPass(pipeline),
  composite_shader_(new ShaderProgram)
{
  std::string vertex_shader (Resources::lookup_shader(Resources::shaders_uber_shaders_composite_compose_vert));
  std::string fragment_shader(Resources::lookup_shader(Resources::shaders_uber_shaders_composite_compose_frag));

  composite_shader_->create_from_sources(vertex_shader, fragment_shader);

  print_shaders("debug", "composite.txt");
}

////////////////////////////////////////////////////////////////////////////////

CompositePass::~CompositePass() {

  if (composite_shader_) {
    delete composite_shader_;
  }

}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::create(RenderContext const& ctx,
    PipelineConfiguration const& config, std::vector<std::pair<BufferComponent,
    scm::gl::sampler_state_desc>> const& layers) {

  Pass::create(ctx, config, layers);

}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void CompositePass::rendering(SerializedScene const& scene,
                                            RenderContext const& ctx,
                                            CameraMode eye,
                                            Camera const& camera,
                                            FrameBufferObject* target) {
  
  init_ressources(ctx);

  ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
  {
    // gather input textures and set uniforms
    Pass::set_camera_matrices(*composite_shader_, camera, pipeline_->get_current_scene(eye), eye, ctx);

    auto input_tex(inputs_[Pipeline::shading]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
    auto normal_tex(inputs_[Pipeline::geometry]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
    auto depth_tex(inputs_[Pipeline::geometry]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_depth_buffer());

    composite_shader_->set_uniform(ctx, input_tex, "gua_color_gbuffer_in");
    composite_shader_->set_uniform(ctx, normal_tex, "gua_normal_gbuffer_in");
    composite_shader_->set_uniform(ctx, depth_tex, "gua_depth_gbuffer_in");

    composite_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_width");
    composite_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_height");

    composite_shader_->use(ctx);
    {
      fullscreen_quad_->draw(ctx.render_context);
    }
    composite_shader_->unuse(ctx);
  }

  ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::init_ressources(RenderContext const& ctx) {

  if (!depth_stencil_state_) {
    depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);
  }

  if (!fullscreen_quad_) {
    fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(ctx.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));
  }
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ LayerMapping const* CompositePass::get_gbuffer_mapping() const {
  throw std::runtime_error("no gbuffer mapping available for composite pass");
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::print_shaders(std::string const& directory,
                               std::string const& name) const  {
  composite_shader_->save_to_file(directory, name + "/composite_shader");
}

////////////////////////////////////////////////////////////////////////////////

bool CompositePass::pre_compile_shaders(RenderContext const& ctx) {

    if (composite_shader_)            return composite_shader_->upload_to(ctx);

    return false;
}

}
