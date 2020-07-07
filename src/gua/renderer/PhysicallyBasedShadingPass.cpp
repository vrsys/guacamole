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
#include <gua/renderer/PhysicallyBasedShadingPass.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
//#include <gua/node/PointLightNode.hpp>
//#include <gua/node/SpotLightNode.hpp>
#include <gua/utils/Logger.hpp>

namespace gua
{
namespace
{
////////////////////////////////////////////////////////////////////////////////

void lighting(PipelinePass& pass, PipelinePassDescription const&, Pipeline& pipe, bool render_multiview, bool use_hardware_mvr)
{
#if 0
  auto const& ctx(pipe.get_context());
  auto gl_program(ctx.render_context->current_program());

  // point lights
  gl_program->uniform_subroutine( scm::gl::STAGE_VERTEX_SHADER,
                                  "compute_light",
                                  "gua_calculate_point_light");
  gl_program->uniform_subroutine( scm::gl::STAGE_FRAGMENT_SHADER,
                                  "compute_light",
                                  "gua_calculate_point_light");

  auto light_sphere =
      std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_sphere_proxy"));
  for (auto const& l : pipe.get_scene().nodes[std::type_index(typeid(node::PointLightNode))]) {
    auto light(reinterpret_cast<node::PointLightNode*>(l));

    gl_program->uniform("gua_model_matrix",
        0, light->get_cached_world_transform());
    gl_program->uniform("gua_light_diffuse_enable",
        0, light->data.get_enable_diffuse_shading());
    gl_program->uniform("gua_light_specular_enable",
        0, light->data.get_enable_specular_shading());
    gl_program->uniform("gua_light_color",
        0, light->data.get_color().vec3f());
    gl_program->uniform("gua_light_falloff",
        0, light->data.get_falloff());
    gl_program->uniform("gua_light_brightness",
        0, light->data.get_brightness());
    gl_program->uniform("gua_light_casts_shadow", 0, false);

    ctx.render_context->apply();
    light_sphere->draw(ctx);
  }

  gl_program->uniform_subroutine( scm::gl::STAGE_VERTEX_SHADER,
                               "compute_light",
                               "gua_calculate_spot_light");
  gl_program->uniform_subroutine( scm::gl::STAGE_FRAGMENT_SHADER,
                               "compute_light",
                               "gua_calculate_spot_light");

  // spot lights
  auto light_cone =
      std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_cone_proxy"));

  for (auto const& l : pipe.get_scene().nodes[std::type_index(typeid(node::SpotLightNode))]) {
    auto light(reinterpret_cast<node::SpotLightNode*>(l));

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

    gl_program->uniform("gua_model_matrix",
        0, light->get_cached_world_transform());
    gl_program->uniform("gua_light_diffuse_enable",
        0, light->data.get_enable_diffuse_shading());
    gl_program->uniform("gua_light_specular_enable",
        0, light->data.get_enable_specular_shading());
    gl_program->uniform("gua_light_color",
        0, light->data.get_color().vec3f());
    gl_program->uniform("gua_light_falloff",
        0, light->data.get_falloff());
    gl_program->uniform("gua_light_brightness",
        0, light->data.get_brightness());
    gl_program->uniform("gua_light_softness",
        0, light->data.get_softness());
    gl_program->uniform("gua_light_casts_shadow",
        0, false);
    //     0, light->data.get_enable_shadows());
    ctx.render_context->apply();
    light_cone->draw(ctx);
  }
#endif
}

} // namespace

////////////////////////////////////////////////////////////////////////////////

PhysicallyBasedShadingPassDescription::PhysicallyBasedShadingPassDescription() : PipelinePassDescription()
{
    // here we assume, that the emissive pass was run previously
    // so we don't swap and don't clear the colorbuffer
    vertex_shader_ = "shaders/physically_based_shading.vert";
    fragment_shader_ = "shaders/physically_based_shading.frag";
    private_.name_ = "PhysicallyBasedShadingPass";

    private_.needs_color_buffer_as_input_ = false; // don't ping pong the color buffer
    private_.writes_only_color_buffer_ = true;     // we write out a color
    private_.rendermode_ = RenderMode::Callback;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(false, false));
    private_.blend_state_desc_ = boost::make_optional(scm::gl::blend_state_desc(scm::gl::blend_ops(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE)));
    private_.rasterizer_state_desc_ = boost::make_optional(scm::gl::rasterizer_state_desc(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT));

    private_.process_ = lighting;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> PhysicallyBasedShadingPassDescription::make_copy() const { return std::make_shared<PhysicallyBasedShadingPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass PhysicallyBasedShadingPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
