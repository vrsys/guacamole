// class header
#include <gua/renderer/LightVisibilityPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShadowMapBuffer.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/node/PointLightNode.hpp>
#include <gua/node/SpotLightNode.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/renderer/LightTable.hpp>

namespace gua {

namespace {

void lighting(PipelinePass& pass, PipelinePassDescription const& , Pipeline& pipe) {
  auto const& ctx(pipe.get_context());
  auto gl_program(ctx.render_context->current_program());

  // collect lights

  LightTable::array_type lights;

  // point lights
  for (auto const& l : pipe.get_scene().nodes[std::type_index(typeid(node::PointLightNode))]) {
    auto light(reinterpret_cast<node::PointLightNode*>(l));

    LightTable::LightBlock light_block {};

    light_block.model_matrix    = light->get_cached_world_transform();
    light_block.color           = math::vec4(light->data.get_color().vec3(), 0.f);
    light_block.falloff         = light->data.get_falloff();
    light_block.brightness      = light->data.get_brightness();
    light_block.softness        = 0;
    light_block.type            = 0;
    light_block.diffuse_enable  = light->data.get_enable_diffuse_shading();
    light_block.specular_enable = light->data.get_enable_specular_shading();
    light_block.casts_shadow    = 0;

    lights.push_back(light_block);
  }

  // spot lights
  for (auto const& l : pipe.get_scene().nodes[std::type_index(typeid(node::SpotLightNode))]) {
    auto light(reinterpret_cast<node::SpotLightNode*>(l));

    LightTable::LightBlock light_block {};

    if (light->data.get_enable_shadows()) {
      // not implemented yet
    }

    light_block.model_matrix    = light->get_cached_world_transform();
    light_block.color           = math::vec4(light->data.get_color().vec3(), 0.f);
    light_block.falloff         = light->data.get_falloff();
    light_block.brightness      = light->data.get_brightness();
    light_block.softness        = light->data.get_softness();
    light_block.type            = 1;
    light_block.diffuse_enable  = light->data.get_enable_diffuse_shading();
    light_block.specular_enable = light->data.get_enable_specular_shading();
    light_block.casts_shadow    = 0; //light->data.get_enable_shadows();

    lights.push_back(light_block);
  }

  // proxy geometries
  auto light_sphere =
      std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_sphere_proxy"));
  auto light_cone =
      std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_cone_proxy"));

  pipe.get_light_table().invalidate(ctx, pipe.get_camera().config.get_resolution(), lights);

  // draw lights
  int lid = 0;
  for (const auto& light : lights) {
    gl_program->uniform("gua_model_matrix", 0, light.model_matrix);
    gl_program->uniform("light_id", 0, lid++);
    ctx.render_context->bind_image(pipe.get_light_table().get_light_bitset()->get_buffer(ctx), 
                                   scm::gl::FORMAT_R_32UI, scm::gl::ACCESS_READ_WRITE, 0, 0, 0);
    ctx.render_context->apply();

    if (light.type == 0)
      light_sphere->draw(ctx);
    else if (light.type == 1)
      light_cone->draw(ctx);
  }

}

}

LightVisibilityPassDescription::LightVisibilityPassDescription()
  : PipelinePassDescription() {
  vertex_shader_ = "shaders/light_visibility.vert";
  fragment_shader_ = "shaders/light_visibility.frag";
  needs_color_buffer_as_input_ = false; // don't ping pong the color buffer
  writes_only_color_buffer_ = false; // we don't write out a color
  doClear_ = false;
  rendermode_ = RenderMode::Callback;

  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));

  rasterizer_state_ = boost::make_optional(scm::gl::rasterizer_state_desc(
        scm::gl::FILL_SOLID, scm::gl::CULL_FRONT));

  process_ = lighting;
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* LightVisibilityPassDescription::make_copy() const {
  return new LightVisibilityPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass LightVisibilityPassDescription::make_pass(RenderContext const& ctx)
{
  PipelinePass pass{*this, ctx};
  return pass;
}

}
