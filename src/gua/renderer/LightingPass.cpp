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
#include <gua/renderer/LightingPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShadowMapBuffer.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

LightingPassDescription::LightingPassDescription()
  : PipelinePassDescription() {
  // here we assume, that the emissive pass was run previously
  // so we don't swap and don't clear the colorbuffer
  needs_color_buffer_as_input_ = false; // don't ping pong the color buffer
  writes_only_color_buffer_ = true; // we write out a color
  doClear_ = false;
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* LightingPassDescription::make_copy() const {
  return new LightingPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass LightingPassDescription::make_pass(
    RenderContext const& ctx) const {
  PipelinePass pass{};

  pass.shader_ = std::make_shared<ShaderProgram>();
  pass.shader_->create_from_sources(
      Resources::lookup_shader(Resources::shaders_lighting_vert),
      Resources::lookup_shader(Resources::shaders_lighting_frag));

  pass.needs_color_buffer_as_input_ = needs_color_buffer_as_input_;
  pass.writes_only_color_buffer_    = writes_only_color_buffer_;
  pass.doClear_                     = doClear_;
  pass.rendermode_                  = rendermode_;

  pass.rasterizer_state_ = ctx.render_device
      ->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT);
  pass.depth_stencil_state_ =
      ctx.render_device->create_depth_stencil_state(false, false);
  pass.blend_state_ = ctx.render_device->create_blend_state(true,
                                                            scm::gl::FUNC_ONE,
                                                            scm::gl::FUNC_ONE,
                                                            scm::gl::FUNC_ONE,
                                                            scm::gl::FUNC_ONE);

  pass.process_ = [](
      PipelinePass & pass, PipelinePassDescription*, Pipeline & pipe) {

    auto light_sphere =
        GeometryDatabase::instance()->lookup("gua_light_sphere_proxy");
    auto light_cone =
        GeometryDatabase::instance()->lookup("gua_light_cone_proxy");

    auto const& ctx(pipe.get_context());

    // init resources
    // bind gbuffer
    pipe.get_gbuffer().bind(ctx, &pass);
    pipe.get_gbuffer().set_viewport(ctx);
    if (pass.doClear_)
      pipe.get_gbuffer().clear_color(ctx);

    // set state
    if (pass.depth_stencil_state_)
      ctx.render_context->set_depth_stencil_state(pass.depth_stencil_state_);
    if (pass.blend_state_)
      ctx.render_context->set_blend_state(pass.blend_state_);
    if (pass.rasterizer_state_)
      ctx.render_context->set_rasterizer_state(pass.rasterizer_state_);

    // draw proxy geometries for light sources
    pass.shader_->use(ctx);
    pipe.bind_gbuffer_input(pass.shader_);

    // point lights
    // --------------------------------------------------------------

    pass.shader_->set_subroutine(ctx,
                                 scm::gl::STAGE_VERTEX_SHADER,
                                 "compute_light",
                                 "gua_calculate_point_light");
    pass.shader_->set_subroutine(ctx,
                                 scm::gl::STAGE_FRAGMENT_SHADER,
                                 "compute_light",
                                 "gua_calculate_point_light");

    for (auto const& light : pipe.get_scene().point_lights_) {
      pass.shader_->set_uniform(
          ctx, light->get_cached_world_transform(), "gua_model_matrix");
      pass.shader_->set_uniform(ctx,
                                light->data.get_enable_diffuse_shading(),
                                "gua_light_diffuse_enable");
      pass.shader_->set_uniform(ctx,
                                light->data.get_enable_specular_shading(),
                                "gua_light_specular_enable");
      pass.shader_
          ->set_uniform(ctx, light->data.get_color().vec3(), "gua_light_color");
      pass.shader_
          ->set_uniform(ctx, light->data.get_falloff(), "gua_light_falloff");
      pass.shader_->set_uniform(ctx, false, "gua_light_casts_shadow");

      ctx.render_context->apply();
      light_sphere->draw(ctx);
    }

    pass.shader_->set_subroutine(ctx,
                                 scm::gl::STAGE_VERTEX_SHADER,
                                 "compute_light",
                                 "gua_calculate_spot_light");
    pass.shader_->set_subroutine(ctx,
                                 scm::gl::STAGE_FRAGMENT_SHADER,
                                 "compute_light",
                                 "gua_calculate_spot_light");

    // spot lights
    // ---------------------------------------------------------------

    for (auto const& light : pipe.get_scene().spot_lights_) {

      if (light->data.get_enable_shadows()) {
        // ctx.render_context->reset_state_objects();

        // shadow_map_.render(
        //   pipe, light->get_cached_world_transform(),
        // light->data.get_shadow_map_size()
        // );

        // shader_->use(ctx);
        // pipe.get_gbuffer().bind(ctx, this);
        // pipe.get_gbuffer().set_viewport(ctx);

        // ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
        // ctx.render_context->set_rasterizer_state(rasterizer_state_);
        // ctx.render_context->set_blend_state(blend_state_);

        // float shadow_map_portion(
        //   1.f * light->data.get_shadow_map_size() /
        //   shadow_map_.get_buffer()->get_width()
        // );

        // shader_->set_uniform(ctx, shadow_map_portion,
        // "gua_light_shadow_map_portion");
        // shader_->set_uniform(ctx,
        // shadow_map_.get_projection_view_matrices()[0],
        // "gua_light_shadow_map_projection_view_matrix_0");
        // shader_->set_uniform(ctx,
        // shadow_map_.get_buffer()->get_depth_buffer()->get_handle(ctx),
        // "gua_light_shadow_map");
        // shader_->set_uniform(ctx, light->data.get_shadow_offset(),
        // "gua_shadow_offset");
      }

      pass.shader_->set_uniform(
          ctx, light->get_cached_world_transform(), "gua_model_matrix");
      pass.shader_->set_uniform(ctx,
                                light->data.get_enable_diffuse_shading(),
                                "gua_light_diffuse_enable");
      pass.shader_->set_uniform(ctx,
                                light->data.get_enable_specular_shading(),
                                "gua_light_specular_enable");
      pass.shader_
          ->set_uniform(ctx, light->data.get_color().vec3(), "gua_light_color");
      pass.shader_
          ->set_uniform(ctx, light->data.get_falloff(), "gua_light_falloff");
      pass.shader_
          ->set_uniform(ctx, light->data.get_softness(), "gua_light_softness");
      pass.shader_->set_uniform(ctx, false, "gua_light_casts_shadow");
      // shader_->set_uniform(ctx, light->data.get_enable_shadows(),
      // "gua_light_casts_shadow");

      ctx.render_context->apply();
      light_cone->draw(ctx);
    }

    pipe.get_gbuffer().unbind(ctx);

    ctx.render_context->reset_state_objects();
  };

  return pass;
}

////////////////////////////////////////////////////////////////////////////////

}
