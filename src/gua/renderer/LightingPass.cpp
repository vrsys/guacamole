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
      shader_(new LightingUberShader),
      light_sphere_(nullptr),
      light_cone_(nullptr),
      serializer_(new Serializer),
      shadow_map_mesh_shader_(new ShadowMapMeshShader),
      shadow_map_nurbs_shader_(new ShadowMapNURBSShader),
      shadow_map_(nullptr),
      shadow_map_projection_view_matrix_(math::mat4::identity()) {
    light_sphere_ = GeometryDatabase::instance()->lookup("gua_light_sphere_proxy");
    light_cone_ = GeometryDatabase::instance()->lookup("gua_light_cone_proxy");
}


////////////////////////////////////////////////////////////////////////////////

LightingPass::~LightingPass() {
    if (shadow_map_mesh_shader_)
      delete shadow_map_mesh_shader_;
    if (shadow_map_nurbs_shader_)
      delete shadow_map_nurbs_shader_;
    if (serializer_)
      delete serializer_;
    if (shader_)
      delete shader_;
}


////////////////////////////////////////////////////////////////////////////////

void LightingPass::apply_material_mapping(
    std::set<std::string> const& material_names,
    std::vector<LayerMapping const*> const& inputs) const {
  shader_->create(material_names, inputs);
}

////////////////////////////////////////////////////////////////////////////////

LayerMapping const* LightingPass::get_gbuffer_mapping() const {
  return shader_->get_gbuffer_mapping();
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void LightingPass::print_shaders(std::string const& directory,
                                               std::string const& name) const {
  shader_->save_to_file(directory, name + "/lighting");
  shadow_map_mesh_shader_->save_to_file(directory, name + "/shadow/mesh");
  shadow_map_nurbs_shader_->save_to_file(directory, name + "/shadow/nurbs");
}

////////////////////////////////////////////////////////////////////////////////

bool LightingPass::pre_compile_shaders(RenderContext const& ctx) {
    if (shader_) shader_->upload_to(ctx);
    if (shadow_map_mesh_shader_) shadow_map_mesh_shader_->upload_to(ctx);
    // if (shadow_map_nurbs_shader_) shadow_map_nurbs_shader_->upload_to(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void LightingPass::rendering(SerializedScene const& scene,
                             RenderContext const& ctx,
                             CameraMode eye,
                             Camera const& camera,
                             FrameBufferObject* target) {
    if (!depth_stencil_state_)
        depth_stencil_state_ =
            ctx.render_device->create_depth_stencil_state(false, false);

    if (!rasterizer_state_)
        rasterizer_state_ = ctx.render_device
            ->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT);

    if (!blend_state_)
        blend_state_ = ctx.render_device->create_blend_state(true,
                                                             scm::gl::FUNC_ONE,
                                                             scm::gl::FUNC_ONE,
                                                             scm::gl::FUNC_ONE,
                                                             scm::gl::FUNC_ONE);

    ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
    ctx.render_context->set_rasterizer_state(rasterizer_state_);
    ctx.render_context->set_blend_state(blend_state_);

    shader_->set_material_uniforms(
        scene.materials_, ShadingModel::LIGHTING_STAGE, ctx);
    shader_->set_subroutine(ctx,
                            scm::gl::STAGE_VERTEX_SHADER,
                            "compute_light",
                            "gua_calculate_point_light");

    shader_->set_subroutine(ctx,
                            scm::gl::STAGE_FRAGMENT_SHADER,
                            "compute_light",
                            "gua_calculate_point_light");

    shader_->use(ctx);

    Pass::bind_inputs(*shader_, eye, ctx);
    Pass::set_camera_matrices(
        *shader_, camera, pipeline_->get_current_scene(eye), eye, ctx);

    for (auto const& light : scene.point_lights_) {
        shader_->set_uniform(
            ctx, light.data.get_enable_diffuse_shading(), "gua_light_diffuse_enable");
        shader_->set_uniform(ctx,
                             light.data.get_enable_specular_shading(),
                             "gua_light_specular_enable");
        shader_->set_uniform(ctx, light.transform, "gua_model_matrix");
        shader_->set_uniform(ctx, light.data.get_color().vec3(), "gua_light_color");
        shader_->set_uniform(ctx, light.data.get_falloff(), "gua_light_falloff");
        shader_->set_uniform(ctx, false, "gua_light_casts_shadow");
        light_sphere_->draw(ctx);
    }

    shader_->set_subroutine(ctx,
                            scm::gl::STAGE_VERTEX_SHADER,
                            "compute_light",
                            "gua_calculate_spot_light");

    shader_->set_subroutine(ctx,
                            scm::gl::STAGE_FRAGMENT_SHADER,
                            "compute_light",
                            "gua_calculate_spot_light");

    for (auto const& light : scene.spot_lights_) {
        if (light.data.get_enable_shadows()) {
            shader_->unuse(ctx);
            target->unbind(ctx);
            ctx.render_context->reset_state_objects();

            render_shadow_map(ctx, camera, light.transform, light.data.get_shadow_map_size());

            shader_->use(ctx);
            target->bind(ctx);

            ctx.render_context->set_viewport(scm::gl::viewport(
                math::vec2(0, 0),
                math::vec2(target->width(), target->height())));

            ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
            ctx.render_context->set_rasterizer_state(rasterizer_state_);
            ctx.render_context->set_blend_state(blend_state_);

            shader_->set_uniform(
                ctx, shadow_map_->get_depth_buffer(), "gua_light_shadow_map");

            float shadow_map_portion(1.f * light.data.get_shadow_map_size() / shadow_map_->width());
            shader_->set_uniform(
                ctx, shadow_map_portion, "gua_light_shadow_map_portion");

            shader_->set_uniform(ctx,
                                 shadow_map_projection_view_matrix_,
                                 "gua_light_shadow_map_projection_view_matrix");

        }

        shader_->set_uniform(
            ctx, light.data.get_enable_shadows(), "gua_light_casts_shadow");
        shader_->set_uniform(
            ctx, light.data.get_enable_diffuse_shading(), "gua_light_diffuse_enable");
        shader_->set_uniform(ctx,
                             light.data.get_enable_specular_shading(),
                             "gua_light_specular_enable");
        shader_->set_uniform(ctx, light.transform, "gua_model_matrix");
        shader_->set_uniform(ctx, light.data.get_color().vec3(), "gua_light_color");
        shader_->set_uniform(ctx, light.data.get_falloff(), "gua_light_falloff");
        shader_->set_uniform(ctx, light.data.get_softness(), "gua_light_softness");
        shader_->set_uniform(
            ctx, light.data.get_shadow_offset(), "gua_shadow_offset");
        light_cone_->draw(ctx);
    }

    shader_->unuse(ctx);

    ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

void LightingPass::render_shadow_map(RenderContext const & ctx,
                                     Camera const& scene_camera,
                                     math::mat4 const & transform,
                                     unsigned map_size) {

    //check whether shadow map size is sufficient
    if (shadow_map_ && shadow_map_->width() < map_size) {
      shadow_map_->remove_buffers(ctx);
      delete shadow_map_;
      shadow_map_ = nullptr;
    }

    if (!shadow_map_) {
        scm::gl::sampler_state_desc state;
        state._compare_mode = scm::gl::TEXCOMPARE_COMPARE_REF_TO_TEXTURE;

#if GUA_COMPILER == GUA_COMPILER_MSVC&& SCM_COMPILER_VER <= 1700
        std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> >
            gbuffer_desc;
        gbuffer_desc.push_back(std::make_pair(BufferComponent::DEPTH_16, state));
        shadow_map_ = new GBuffer(gbuffer_desc, map_size, map_size);
#else
        shadow_map_ = new GBuffer({
          { BufferComponent::DEPTH_16, state }
        },
                                  map_size,
                                  map_size);
#endif
        shadow_map_->create(ctx);
    }

    shadow_map_->bind(ctx);

    ctx.render_context->set_viewport(scm::gl::viewport(
        math::vec2(0, 0),
        math::vec2(map_size, map_size)));

    shadow_map_->clear_depth_stencil_buffer(ctx);

    math::mat4 screen_transform(scm::math::make_translation(0.f, 0.f, -1.f));
    screen_transform = transform * screen_transform;

    Frustum shadow_frustum(transform,
                           screen_transform,
                           pipeline_->config.near_clip(),
                           pipeline_->config.far_clip());

    SerializedScene scene;
    serializer_->check(&scene,
                       pipeline_->get_current_graph(),
                       Camera("", "", scene_camera.render_mask),
                       shadow_frustum,
                       false,
                       false,
                       true);

    shadow_map_projection_view_matrix_ =
        shadow_frustum.get_projection() * shadow_frustum.get_view();

    shadow_map_mesh_shader_->set_uniform(
        ctx, shadow_map_projection_view_matrix_, "gua_projection_view_matrix");
    shadow_map_mesh_shader_->use(ctx);

    // let derived class render all geometries
    if (!shadow_map_depth_stencil_state_)
        shadow_map_depth_stencil_state_ =
            ctx.render_device->create_depth_stencil_state(true, true);

    if (!shadow_map_rasterizer_state_)
        shadow_map_rasterizer_state_ = ctx.render_device
            ->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);

    ctx.render_context->set_depth_stencil_state(shadow_map_depth_stencil_state_);
    ctx.render_context->set_rasterizer_state(shadow_map_rasterizer_state_);

    for (auto const& node : scene.meshnodes_) {
        auto geometry = GeometryDatabase::instance()->lookup(node.data.get_geometry());

        if (geometry) {
            shadow_map_mesh_shader_->set_uniform(
                ctx, node.transform, "gua_model_matrix");
            geometry->draw(ctx);
        }
    }

    ctx.render_context->reset_state_objects();

    shadow_map_mesh_shader_->unuse(ctx);

    shadow_map_->unbind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

}
