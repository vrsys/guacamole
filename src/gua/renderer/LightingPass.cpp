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

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/LightingUberShader.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/renderer/Serializer.hpp>
#include <gua/databases.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

LightingPass::LightingPass(Pipeline* pipeline)
    : GeometryPass(pipeline),
      shadow_map_(pipeline),
      shader_(new LightingUberShader),
      light_sphere_(nullptr),
      light_cone_(nullptr)
  {
    light_sphere_ = GeometryDatabase::instance()->lookup("gua_light_sphere_proxy");
    light_cone_ = GeometryDatabase::instance()->lookup("gua_light_cone_proxy");
}


////////////////////////////////////////////////////////////////////////////////

LightingPass::~LightingPass() {
    if (shader_)
      delete shader_;
}


////////////////////////////////////////////////////////////////////////////////

void LightingPass::apply_material_mapping(
    std::set<std::string> const& material_names,
    std::vector<LayerMapping const*> const& inputs) const {
  shader_->create(material_names, inputs);
  shadow_map_.apply_material_mapping(material_names);
}

////////////////////////////////////////////////////////////////////////////////

LayerMapping const* LightingPass::get_gbuffer_mapping() const {
  return shader_->get_gbuffer_mapping();
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void LightingPass::print_shaders(std::string const& directory,
                                               std::string const& name) const {
  shader_->get_pass(0)->save_to_file(directory, name + "/lighting");
  shadow_map_.print_shaders(directory, name);
}

////////////////////////////////////////////////////////////////////////////////

bool LightingPass::pre_compile_shaders(RenderContext const& ctx) {

    bool success(false);

    if (shader_) success = shader_->upload_to(ctx);
    if (success) success = shadow_map_.pre_compile_shaders(ctx);

    return success;
}

////////////////////////////////////////////////////////////////////////////////

void LightingPass::cleanup(RenderContext const& ctx) {
    if (shader_) shader_->cleanup(ctx);
    shadow_map_.cleanup(ctx);
    GeometryPass::cleanup(ctx);
}

////////////////////////////////////////////////////////////////////////////////


void LightingPass::init_resources(RenderContext const& ctx) {
  if (!initialized_) {
    if (!depth_stencil_state_)
        depth_stencil_state_ =
            ctx.render_device->create_depth_stencil_state(false, false);

    if (!rasterizer_state_front_)
        rasterizer_state_front_ = ctx.render_device
            ->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT);

    if (!rasterizer_state_back_)
        rasterizer_state_back_ = ctx.render_device
            ->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);

    if (!blend_state_)
        blend_state_ = ctx.render_device->create_blend_state(true,
                                                             scm::gl::FUNC_ONE,
                                                             scm::gl::FUNC_ONE,
                                                             scm::gl::FUNC_ONE,
                                                             scm::gl::FUNC_ONE);

    if (!fullscreen_quad_)
      fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(
        ctx.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));
    initialized_ = true;
  }
}

void LightingPass::rendering(SerializedScene const& scene,
                             SceneGraph const& scene_graph,
                             RenderContext const& ctx,
                             CameraMode eye,
                             Camera const& camera,
                             FrameBufferObject* target) {
    init_resources(ctx);

    ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
    ctx.render_context->set_rasterizer_state(rasterizer_state_back_);
    ctx.render_context->set_blend_state(blend_state_);

    shader_->set_material_uniforms(
        scene.materials_, ShadingModel::LIGHTING_STAGE, ctx);
    shader_->get_pass(0)->use(ctx);

    Pass::bind_inputs(*shader_->get_pass(0), eye, ctx);
    Pass::set_camera_matrices(
      *shader_->get_pass(0), camera, pipeline_->get_current_scene(eye), eye, ctx);


    // -------------------------- sun lights -----------------------------------
    shader_->get_pass(0)->set_subroutine(ctx,
                            scm::gl::STAGE_VERTEX_SHADER,
                            "compute_light",
                            "gua_calculate_sun_light");

    shader_->get_pass(0)->set_subroutine(ctx,
                            scm::gl::STAGE_FRAGMENT_SHADER,
                            "compute_light",
                            "gua_calculate_sun_light");


    for (auto const& light : scene.sun_lights_) {

        if (light->data.get_enable_shadows()) {
            shader_->get_pass(0)->unuse(ctx);
            target->unbind(ctx);
            ctx.render_context->reset_state_objects();

            float split_0(0.f), split_1(0.f), split_2(0.f), split_3(0.f), split_4(0.f);

            if (light->data.get_shadow_cascaded_splits().size() == 5) {

                split_0 = light->data.get_shadow_cascaded_splits()[0];
                split_1 = light->data.get_shadow_cascaded_splits()[1];
                split_2 = light->data.get_shadow_cascaded_splits()[2];
                split_3 = light->data.get_shadow_cascaded_splits()[3];
                split_4 = light->data.get_shadow_cascaded_splits()[4];

            } else {
                Logger::LOG_WARNING << "Exactly 5 splits have to be defined for cascaded shadow maps!" << std::endl;
            }

            shadow_map_.render_cascaded(ctx, scene_graph, scene.center_of_interest, scene.frustum, camera,
                                        light->get_cached_world_transform(),
                                        light->data.get_shadow_map_size(),
                                        split_0, split_1, split_2, split_3, split_4,
                                        light->data.get_shadow_near_clipping_in_sun_direction());

            shader_->get_pass(0)->use(ctx);
            target->bind(ctx);

            ctx.render_context->set_viewport(scm::gl::viewport(
                math::vec2(0, 0),
                math::vec2(float(target->width()), float(target->height()))));

            ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
            ctx.render_context->set_rasterizer_state(rasterizer_state_back_);
            ctx.render_context->set_blend_state(blend_state_);

            shader_->set_uniform(ctx, shadow_map_.get_buffer()->get_depth_buffer(), "gua_light_shadow_map");

            float shadow_map_portion(1.f * light->data.get_shadow_map_size() / shadow_map_.get_buffer()->width());
            shader_->set_uniform(ctx, shadow_map_portion, "gua_light_shadow_map_portion");

            for (int i(0); i<4; ++i) {
              shader_->set_uniform(ctx, shadow_map_.get_projection_view_matrices()[i], "gua_light_shadow_map_projection_view_matrix_" + string_utils::to_string(i));
            }
        }

        shader_->set_uniform(
            ctx, light->data.get_enable_diffuse_shading(), "gua_light_diffuse_enable");
        shader_->set_uniform(ctx,
                             light->data.get_enable_specular_shading(),
                             "gua_light_specular_enable");
        shader_->set_uniform(ctx, light->get_cached_world_transform(), "gua_model_matrix");
        shader_->set_uniform(ctx, light->data.get_color().vec3(), "gua_light_color");
        shader_->set_uniform(ctx, light->data.get_enable_shadows(), "gua_light_casts_shadow");
        shader_->set_uniform(
            ctx, light->data.get_shadow_offset(), "gua_shadow_offset");
        fullscreen_quad_->draw(ctx.render_context);
    }

    // ------------------------- point lights ----------------------------------

    ctx.render_context->set_rasterizer_state(rasterizer_state_front_);

    shader_->get_pass(0)->set_subroutine(ctx,
                            scm::gl::STAGE_VERTEX_SHADER,
                            "compute_light",
                            "gua_calculate_point_light");

    shader_->get_pass(0)->set_subroutine(ctx,
                            scm::gl::STAGE_FRAGMENT_SHADER,
                            "compute_light",
                            "gua_calculate_point_light");


    for (auto const& light : scene.point_lights_) {
        shader_->set_uniform(
            ctx, light->data.get_enable_diffuse_shading(), "gua_light_diffuse_enable");
        shader_->set_uniform(ctx,
                             light->data.get_enable_specular_shading(),
                             "gua_light_specular_enable");
        shader_->set_uniform(ctx, light->get_cached_world_transform(), "gua_model_matrix");
        shader_->set_uniform(ctx, light->data.get_color().vec3(), "gua_light_color");
        shader_->set_uniform(ctx, light->data.get_falloff(), "gua_light_falloff");
        shader_->set_uniform(ctx, false, "gua_light_casts_shadow");
        light_sphere_->draw(ctx);
    }


    // -------------------------- spot lights ----------------------------------
    shader_->get_pass(0)->set_subroutine(ctx,
                            scm::gl::STAGE_VERTEX_SHADER,
                            "compute_light",
                            "gua_calculate_spot_light");

    shader_->get_pass(0)->set_subroutine(ctx,
                            scm::gl::STAGE_FRAGMENT_SHADER,
                            "compute_light",
                            "gua_calculate_spot_light");

    for (auto const& light : scene.spot_lights_) {
        if (light->data.get_enable_shadows()) {
            shader_->get_pass(0)->unuse(ctx);
            target->unbind(ctx);
            ctx.render_context->reset_state_objects();

            shadow_map_.render(ctx, scene_graph, scene.center_of_interest, camera, light->get_cached_world_transform(), light->data.get_shadow_map_size());

            shader_->get_pass(0)->use(ctx);
            target->bind(ctx);

            ctx.render_context->set_viewport(scm::gl::viewport(
                math::vec2(0, 0),
                math::vec2(float(target->width()), float(target->height()))));

            ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
            ctx.render_context->set_rasterizer_state(rasterizer_state_front_);
            ctx.render_context->set_blend_state(blend_state_);

            shader_->set_uniform(ctx, shadow_map_.get_buffer()->get_depth_buffer(), "gua_light_shadow_map");

            float shadow_map_portion(1.f * light->data.get_shadow_map_size() / shadow_map_.get_buffer()->width());
            shader_->set_uniform(ctx, shadow_map_portion, "gua_light_shadow_map_portion");
            shader_->set_uniform(ctx, shadow_map_.get_projection_view_matrices()[0], "gua_light_shadow_map_projection_view_matrix_0");
        }

        shader_->set_uniform(
            ctx, light->data.get_enable_shadows(), "gua_light_casts_shadow");
        shader_->set_uniform(
            ctx, light->data.get_enable_diffuse_shading(), "gua_light_diffuse_enable");
        shader_->set_uniform(ctx,
                             light->data.get_enable_specular_shading(),
                             "gua_light_specular_enable");
        shader_->set_uniform(ctx, light->get_cached_world_transform(), "gua_model_matrix");
        shader_->set_uniform(ctx, light->data.get_color().vec3(), "gua_light_color");
        shader_->set_uniform(ctx, light->data.get_falloff(), "gua_light_falloff");
        shader_->set_uniform(ctx, light->data.get_softness(), "gua_light_softness");
        shader_->set_uniform(
            ctx, light->data.get_shadow_offset(), "gua_shadow_offset");
        light_cone_->draw(ctx);
    }

    shader_->get_pass(0)->unuse(ctx);

    ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

}
