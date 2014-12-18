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
  std::vector<math::mat4> transforms;
  LightTable::array_type lights;

  // point lights
  for (auto const& l : pipe.get_scene().nodes[std::type_index(typeid(node::PointLightNode))]) {
    auto light(reinterpret_cast<node::PointLightNode*>(l));

    auto model_mat = light->get_cached_world_transform();

    math::vec3 light_position = model_mat * math::vec4(0.0, 0.0, 0.0, 1.0);
    float light_radius = scm::math::length(light_position - math::vec3(model_mat * math::vec4(0.0, 0.0, 1.0, 1.0)));

    LightTable::LightBlock light_block {};

    light_block.position_and_radius = math::vec4(light_position, light_radius);
    light_block.beam_direction_and_half_angle = math::vec4(0.f, 0.f, 0.f, 0.f);
    light_block.color           = math::vec4(light->data.get_color().vec3(), 0.f);
    light_block.falloff         = light->data.get_falloff();
    light_block.brightness      = light->data.get_brightness();
    light_block.softness        = 0;
    light_block.type            = 0;
    light_block.diffuse_enable  = light->data.get_enable_diffuse_shading();
    light_block.specular_enable = light->data.get_enable_specular_shading();
    light_block.casts_shadow    = 0;

    lights.push_back(light_block);
    transforms.push_back(model_mat);
  }

  // spot lights
  for (auto const& l : pipe.get_scene().nodes[std::type_index(typeid(node::SpotLightNode))]) {
    auto light(reinterpret_cast<node::SpotLightNode*>(l));

    auto model_mat = light->get_cached_world_transform();

    math::vec3 light_position = model_mat * math::vec4(0.0, 0.0, 0.0, 1.0);
    float light_radius = scm::math::length(light_position - math::vec3(model_mat * math::vec4(0.0, 0.0, 1.0, 1.0)));
    math::vec3 beam_direction = math::vec3(model_mat * math::vec4(0.0, 0.0, -1.0, 1.0)) - light_position;
    float half_beam_angle = scm::math::dot(scm::math::normalize(math::vec3(model_mat * math::vec4(0.0, 0.5, -1.0, 0.0))),
                                           scm::math::normalize(beam_direction));

    LightTable::LightBlock light_block {};

    if (light->data.get_enable_shadows()) {
      // not implemented yet
    }

    light_block.position_and_radius = math::vec4(light_position, light_radius);
    light_block.beam_direction_and_half_angle = math::vec4(beam_direction, half_beam_angle);
    light_block.color           = math::vec4(light->data.get_color().vec3(), 0.f);
    light_block.falloff         = light->data.get_falloff();
    light_block.brightness      = light->data.get_brightness();
    light_block.softness        = light->data.get_softness();
    light_block.type            = 1;
    light_block.diffuse_enable  = light->data.get_enable_diffuse_shading();
    light_block.specular_enable = light->data.get_enable_specular_shading();
    light_block.casts_shadow    = 0; //light->data.get_enable_shadows();

    lights.push_back(light_block);
    transforms.push_back(model_mat);
  }

  // proxy geometries
  auto light_sphere =
      std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_sphere_proxy"));
  auto light_cone =
      std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_cone_proxy"));

  pipe.get_light_table().invalidate(ctx, pipe.get_camera().config.get_resolution(), lights);

  // draw lights
  for (size_t i = 0; i < lights.size(); ++i) {
    gl_program->uniform("gua_model_matrix", 0, transforms[i]);
    gl_program->uniform("light_id", 0, int(i));
    ctx.render_context->bind_image(pipe.get_light_table().get_light_bitset()->get_buffer(ctx), 
                                   scm::gl::FORMAT_R_32UI, scm::gl::ACCESS_READ_WRITE, 0, 0, 0);
    ctx.render_context->apply();

    if (lights[i].type == 0)
      light_sphere->draw(ctx);
    else if (lights[i].type == 1)
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
