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
#include <gua/renderer/GBufferPass.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GBufferMeshUberShader.hpp>
#include <gua/renderer/GBufferNURBSUberShader.hpp>
#include <gua/renderer/GBufferVideo3DUberShader.hpp>
#include <gua/renderer/MeshLoader.hpp>
#include <gua/renderer/Video3D.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Video3DDatabase.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

// #define DEBUG_XFB_OUTPUT

namespace gua {

////////////////////////////////////////////////////////////////////////////////

GBufferPass::GBufferPass(Pipeline* pipeline)
    : GeometryPass(pipeline),
      mesh_shader_(new GBufferMeshUberShader),
      nurbs_shader_(new GBufferNURBSUberShader),
      video3D_shader_(new GBufferVideo3DUberShader),
      bfc_rasterizer_state_(),
      no_bfc_rasterizer_state_(),
      bbox_rasterizer_state_(),
      depth_stencil_state_(),
      bounding_box_() {
        MeshLoader mesh_loader;

        bounding_box_ = GeometryDatabase::instance()
                            ->lookup("gua_bounding_box_geometry");
    }

////////////////////////////////////////////////////////////////////////////////

GBufferPass::~GBufferPass() {
    if (mesh_shader_)
      delete mesh_shader_;
    if (nurbs_shader_)
      delete nurbs_shader_;
    if (video3D_shader_)
      delete video3D_shader_;
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::create(
    RenderContext const& ctx,
    std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> > const&
        layers) {

    scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_MIP_NEAREST,
                                      scm::gl::WRAP_MIRRORED_REPEAT,
                                      scm::gl::WRAP_MIRRORED_REPEAT);

    auto tmp(layers);
    tmp.insert(tmp.begin(), std::make_pair(BufferComponent::DEPTH_24, state));

    Pass::create(ctx, tmp);
}


////////////////////////////////////////////////////////////////////////////////

void GBufferPass::print_shaders(std::string const& directory,
                                std::string const& name) const {
    mesh_shader_->save_to_file(directory, name + "/mesh");
    nurbs_shader_->save_to_file(directory, name + "/nurbs");
    video3D_shader_->save_to_file(directory, name + "/video3d");
}

////////////////////////////////////////////////////////////////////////////////

bool GBufferPass::pre_compile_shaders(RenderContext const& ctx) {
    if (mesh_shader_)  return mesh_shader_->upload_to(ctx);
    if (nurbs_shader_) return nurbs_shader_->upload_to(ctx);
    if (video3D_shader_) return video3D_shader_->upload_to(ctx);

    std::cout << "Print Shaders: " << std::endl;
    getchar();
    print_shaders("/tmp","");

    return false;
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::rendering(SerializedScene const& scene,
                            RenderContext const& ctx,
                            CameraMode eye,
                            Camera const& camera,
                            FrameBufferObject* target) {



    if (!depth_stencil_state_)
        depth_stencil_state_ =
            ctx.render_device->create_depth_stencil_state(true, true);

    if (!bfc_rasterizer_state_)
        bfc_rasterizer_state_ = ctx.render_device->create_rasterizer_state(
            pipeline_->config.enable_wireframe() ? scm::gl::FILL_WIREFRAME
                                                 : scm::gl::FILL_SOLID,
            scm::gl::CULL_BACK,
            scm::gl::ORIENT_CCW,
            false);

    if (!no_bfc_rasterizer_state_)
        no_bfc_rasterizer_state_ = ctx.render_device->create_rasterizer_state(
            pipeline_->config.enable_wireframe() ? scm::gl::FILL_WIREFRAME
                                                 : scm::gl::FILL_SOLID,
            scm::gl::CULL_NONE);

    if (!bbox_rasterizer_state_)
        bbox_rasterizer_state_ = ctx.render_device
            ->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);

    ctx.render_context->set_rasterizer_state(
        pipeline_->config.enable_backface_culling() ? bfc_rasterizer_state_
                                                    : no_bfc_rasterizer_state_);

    ctx.render_context->set_depth_stencil_state(depth_stencil_state_);

    ////////////////////////////////////////////////////////////////////
    // set frame-consistent uniforms for mesh ubershader
    ////////////////////////////////////////////////////////////////////
    mesh_shader_->set_material_uniforms(
        scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
    mesh_shader_->set_material_uniforms(
        scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

    mesh_shader_->set_uniform(ctx, scene.enable_global_clipping_plane, "gua_enable_global_clipping_plane");
    mesh_shader_->set_uniform(ctx, scene.global_clipping_plane, "gua_global_clipping_plane");

    Pass::bind_inputs(*mesh_shader_, eye, ctx);
    Pass::set_camera_matrices(*mesh_shader_,
                              camera,
                              pipeline_->get_current_scene(eye),
                              eye,
                              ctx);

    ////////////////////////////////////////////////////////////////////
    // set frame-consistent uniforms for video3d ubershader
    ////////////////////////////////////////////////////////////////////
    video3D_shader_->set_material_uniforms(
      scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
    video3D_shader_->set_material_uniforms(
      scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

    Pass::bind_inputs(*video3D_shader_, eye, ctx);
    Pass::set_camera_matrices(*video3D_shader_,
                              camera,
                              pipeline_->get_current_scene(eye),
                              eye,
                              ctx);

    for (auto const& pass : video3D_shader_->get_pre_passes())
    {
      Pass::bind_inputs(*pass, eye, ctx);
      Pass::set_camera_matrices(*pass,
        camera,
        pipeline_->get_current_scene(eye),
        eye,
        ctx);
    }

    ////////////////////////////////////////////////////////////////////
    // set frame-consistent uniforms for nurbs ubershader
    ////////////////////////////////////////////////////////////////////
    nurbs_shader_->set_material_uniforms(
      scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
    nurbs_shader_->set_material_uniforms(
      scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

    Pass::bind_inputs(nurbs_shader_->get_pre_shader(), eye, ctx);
    Pass::bind_inputs(*nurbs_shader_, eye, ctx);

    Pass::set_camera_matrices(nurbs_shader_->get_pre_shader(),
      camera,
      pipeline_->get_current_scene(eye),
      eye,
      ctx);
    Pass::set_camera_matrices(*nurbs_shader_, camera, pipeline_->get_current_scene(eye), eye, ctx);

    // TODO: add this functionality to NURBS!
    // nurbs_shader_->set_uniform(ctx, scene.enable_global_clipping_plane, "gua_enable_global_clipping_plane");
    // nurbs_shader_->set_uniform(ctx, scene.global_clipping_plane, "gua_global_clipping_plane");

    nurbs_shader_->set_uniform(ctx,
      pipeline_->config.get_max_tesselation(),
      "gua_max_tesselation");

    ////////////////////////////////////////////////////////////////////
    // draw
    ////////////////////////////////////////////////////////////////////
    if (!scene.meshnodes_.empty()) {

        // draw meshes
        mesh_shader_->use(ctx);
        {

            for (auto const& node : scene.meshnodes_) {
                auto geometry =
                    GeometryDatabase::instance()->lookup(node->get_geometry());
                auto material =
                    MaterialDatabase::instance()->lookup(node->get_material());

                if (material && geometry) {
                    mesh_shader_->set_uniform(
                        ctx, material->get_id(), "gua_material_id");
                    mesh_shader_->set_uniform(
                        ctx, node->get_cached_world_transform(), "gua_model_matrix");
                    mesh_shader_->set_uniform(
                        ctx,
                        scm::math::transpose(
                            scm::math::inverse(node->get_cached_world_transform())),
                        "gua_normal_matrix");

                    geometry->draw(ctx);
                }
            }
        }
        mesh_shader_->unuse(ctx);
    }



    if (!scene.textured_quads_.empty()) {

        // draw meshes
        mesh_shader_->use(ctx);
        {

            for (auto const& node : scene.textured_quads_) {
                auto geometry =
                    GeometryDatabase::instance()->lookup("gua_plane_geometry");
                auto material =
                    MaterialDatabase::instance()->lookup("gua_textured_quad");

                std::string texture_name(node->data.get_texture());
                if (node->data.get_is_stereo_texture()) {

                  if (eye == CameraMode::LEFT) {
                    texture_name += "_left";
                  } else if (eye == CameraMode::RIGHT) {
                    texture_name += "_right";
                  }
                }

                if (TextureDatabase::instance()->is_supported(texture_name)) {
                    auto texture =
                        TextureDatabase::instance()->lookup(texture_name);
                    auto mapped_texture(
                        mesh_shader_->get_uniform_mapping()->get_mapping("gua_textured_quad", "texture"));

                    mesh_shader_->set_uniform(ctx, texture, mapped_texture.first, mapped_texture.second);

                    auto mapped_flip_x(
                        mesh_shader_->get_uniform_mapping()->get_mapping("gua_textured_quad", "flip_x"));

                    mesh_shader_->set_uniform(ctx, node->data.get_flip_x(), mapped_flip_x.first, mapped_flip_x.second);

                    auto mapped_flip_y(
                        mesh_shader_->get_uniform_mapping()->get_mapping("gua_textured_quad", "flip_y"));

                    mesh_shader_->set_uniform(ctx, node->data.get_flip_y(), mapped_flip_y.first, mapped_flip_y.second);

                    if (material && geometry) {
                        mesh_shader_->set_uniform(
                            ctx, material->get_id(), "gua_material_id");
                        mesh_shader_->set_uniform(
                            ctx, node->get_scaled_world_transform(), "gua_model_matrix");
                        mesh_shader_->set_uniform(
                            ctx,
                            scm::math::transpose(
                                scm::math::inverse(node->get_scaled_world_transform())),
                            "gua_normal_matrix");

                        geometry->draw(ctx);
                    }
                } else {
                    Logger::LOG_WARNING << "Failed to render TexturedQuad: Texture2D \"" << texture_name << "\" not found!" << std::endl;
                }
            }
        }
        mesh_shader_->unuse(ctx);
    }

    if (!scene.nurbsnodes_.empty()) {
        // draw nurbs

        for (auto const& node : scene.nurbsnodes_)
            {
            auto geometry =
                GeometryDatabase::instance()->lookup(node->get_geometry());
            auto material =
                MaterialDatabase::instance()->lookup(node->get_material());

#ifdef DEBUG_XFB_OUTPUT
            scm::gl::transform_feedback_statistics_query_ptr q = ctx
                .render_device->create_transform_feedback_statistics_query(0);
            ctx.render_context->begin_query(q);
#endif
            // pre-tesselate if necessary
            nurbs_shader_->get_pre_shader().use(ctx);
            {
                nurbs_shader_->get_pre_shader()
                    .set_uniform(ctx, node->get_cached_world_transform(), "gua_model_matrix");
                nurbs_shader_->get_pre_shader().set_uniform(
                    ctx,
                    scm::math::transpose(scm::math::inverse(node->get_cached_world_transform())),
                    "gua_normal_matrix");

                ctx.render_context->apply();
                geometry->predraw(ctx);
            }
            nurbs_shader_->get_pre_shader().unuse(ctx);

#ifdef DEBUG_XFB_OUTPUT
            ctx.render_context->end_query(q);
            ctx.render_context->collect_query_results(q);
            std::cout << q->result()._primitives_generated << " , "
                      << q->result()._primitives_written << std::endl;
#endif

            // invoke tesselation/trim shader for adaptive nurbs rendering
            nurbs_shader_->use(ctx);
            {
                if (material && geometry) {
                    nurbs_shader_->set_uniform(
                        ctx, material->get_id(), "gua_material_id");
                    nurbs_shader_->set_uniform(
                        ctx, node->get_cached_world_transform(), "gua_model_matrix");
                    nurbs_shader_->set_uniform(
                        ctx,
                        scm::math::transpose(
                            scm::math::inverse(node->get_cached_world_transform())),
                        "gua_normal_matrix");

                    geometry->draw(ctx);
                }
            }
            nurbs_shader_->unuse(ctx);
        }
    }

    if (!scene.video3Dnodes_.empty()) {

      for (auto const& node : scene.video3Dnodes_)
      {
        auto normal_matrix = scm::math::transpose(scm::math::inverse(node->get_cached_world_transform()));
        auto model_matrix = node->get_cached_world_transform();

        video3D_shader_->draw(ctx,
                              node->get_ksfile(),
                              node->get_material(),
                              model_matrix,
                              normal_matrix);
      }
    }

    ctx.render_context->set_rasterizer_state(bbox_rasterizer_state_);
    std::shared_ptr<Material> bbox_material;

    if (pipeline_->config.enable_ray_display() ||
        pipeline_->config.enable_bbox_display()) {
        bbox_material =
            MaterialDatabase::instance()->lookup("gua_bounding_box");
    }

    // draw bounding boxes, if desired
    if (pipeline_->config.enable_bbox_display() && !scene.bounding_boxes_.empty()) {

        mesh_shader_->use(ctx);  // re-use mesh_shader

        for (auto const& bbox : scene.bounding_boxes_) {
            math::mat4 bbox_transform(math::mat4::identity());
            auto scale(scm::math::make_scale((bbox.max - bbox.min) * 1.001f));
            auto translation(
                scm::math::make_translation((bbox.max + bbox.min) / 2.f));
            bbox_transform *= translation;
            bbox_transform *= scale;

            mesh_shader_->set_uniform(
                ctx, bbox_material->get_id(), "gua_material_id");
            mesh_shader_->set_uniform(ctx, bbox_transform, "gua_model_matrix");
            mesh_shader_->set_uniform(
                ctx,
                scm::math::transpose(scm::math::inverse(bbox_transform)),
                "gua_normal_matrix");

            bounding_box_->draw(ctx);
        }
        mesh_shader_->unuse(ctx);
    }

    // draw pick rays, if desired
    if (pipeline_->config.enable_ray_display()) {
        mesh_shader_->use(ctx);  // re-use mesh_shader

        for (auto const& ray : scene.rays_) {
            auto geometry = GeometryDatabase::instance()->lookup("gua_ray_geometry");

            mesh_shader_->set_uniform(
                ctx, bbox_material->get_id(), "gua_material_id");
            mesh_shader_->set_uniform(ctx, ray->get_cached_world_transform(), "gua_model_matrix");
            mesh_shader_->set_uniform(
                ctx,
                scm::math::transpose(scm::math::inverse(ray->get_cached_world_transform())),
                "gua_normal_matrix");

            geometry->draw(ctx);
        }
        mesh_shader_->unuse(ctx);
    }

    ctx.render_context->reset_state_objects();
}


////////////////////////////////////////////////////////////////////////////////

void GBufferPass::apply_material_mapping(std::set<std::string> const &
                                         materials) const {
  mesh_shader_->create(materials);
  nurbs_shader_->create(materials);
  video3D_shader_->create(materials);
}


////////////////////////////////////////////////////////////////////////////////

LayerMapping const* GBufferPass::get_gbuffer_mapping() const {
  // todo: dirty to use single Ubershader mapping here -> possible solution:
  // extract gbuffermapping from Ubershader?
  return mesh_shader_->get_gbuffer_mapping();
}

////////////////////////////////////////////////////////////////////////////////

}
