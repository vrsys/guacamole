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
#include <gua/renderer/GBufferVolumeUberShader.hpp>
#include <gua/renderer/MeshLoader.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

#define DEBUG_XFB_OUTPUT

namespace gua {

////////////////////////////////////////////////////////////////////////////////

GBufferPass::GBufferPass(Pipeline* pipeline)
    : GeometryPass(pipeline),
      mesh_shader_(new GBufferMeshUberShader),
	  nurbs_shader_(new GBufferNURBSUberShader),
	  volume_shader_(new GBufferVolumeUberShader),
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
	if (volume_shader_)
		delete volume_shader_;
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::create(
    RenderContext const& ctx,
    PipelineConfiguration const& config,
    std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> > const&
        layers) {

    scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_MIP_NEAREST,
                                      scm::gl::WRAP_CLAMP_TO_EDGE,
                                      scm::gl::WRAP_CLAMP_TO_EDGE);

    auto tmp(layers);
    tmp.insert(tmp.begin(), std::make_pair(BufferComponent::DEPTH_24, state));

    Pass::create(ctx, config, tmp);
}


////////////////////////////////////////////////////////////////////////////////

void GBufferPass::print_shaders(std::string const& directory,
                                std::string const& name) const {
    mesh_shader_->save_to_file(directory, name + "/mesh");
	nurbs_shader_->save_to_file(directory, name + "/nurbs");
	volume_shader_->save_to_file(directory, name + "/volume");
}

////////////////////////////////////////////////////////////////////////////////

bool GBufferPass::pre_compile_shaders(RenderContext const& ctx) {
    if (mesh_shader_)  return mesh_shader_->upload_to(ctx);
	if (nurbs_shader_) return nurbs_shader_->upload_to(ctx);
	if (volume_shader_) return volume_shader_->upload_to(ctx);
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
		
    if (!scene.meshnodes_.empty() || !scene.textured_quads_.empty() || (pipeline_->config.enable_bbox_display() && !scene.bounding_boxes_.empty())) {
        mesh_shader_->set_material_uniforms(
            scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
        mesh_shader_->set_material_uniforms(
            scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

        Pass::bind_inputs(*mesh_shader_, eye, ctx);
        Pass::set_camera_matrices(*mesh_shader_,
                                  camera,
                                  pipeline_->get_current_scene(eye),
                                  eye,
                                  ctx);
    }

    if (!scene.meshnodes_.empty()) {

        // draw meshes
        mesh_shader_->use(ctx);
        {

            for (auto const& node : scene.meshnodes_) {
                auto geometry =
                    GeometryDatabase::instance()->lookup(node.data.get_geometry());
                auto material =
                    MaterialDatabase::instance()->lookup(node.data.get_material());

                if (material && geometry) {
                    mesh_shader_->set_uniform(
                        ctx, material->get_id(), "gua_material_id");
                    mesh_shader_->set_uniform(
                        ctx, node.transform, "gua_model_matrix");
                    mesh_shader_->set_uniform(
                        ctx,
                        scm::math::transpose(
                            scm::math::inverse(node.transform)),
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

                std::string texture_name(node.data.get_texture());
                if (node.data.get_is_stereo_texture()) {

                  if (eye == CameraMode::LEFT) {
                    texture_name += "_left";
                  } else if (eye == CameraMode::RIGHT) {
                    texture_name += "_right";
                  }
                }

                if (TextureDatabase::instance()->is_supported(texture_name)) {
                    auto texture =
                        TextureDatabase::instance()->lookup(texture_name);
                    auto mapped(
                        mesh_shader_->get_uniform_mapping()->get_mapping("gua_textured_quad", "texture"));

                    mesh_shader_->set_uniform(ctx, texture, mapped.first, mapped.second);

                    if (material && geometry) {
                        mesh_shader_->set_uniform(
                            ctx, material->get_id(), "gua_material_id");
                        mesh_shader_->set_uniform(
                            ctx, node.transform, "gua_model_matrix");
                        mesh_shader_->set_uniform(
                            ctx,
                            scm::math::transpose(
                                scm::math::inverse(node.transform)),
                            "gua_normal_matrix");

                        geometry->draw(ctx);
                    }
                } else {
                    WARNING("Failed to render TexturedQuad: Texture2D \"%s\" not found!", texture_name.c_str());
                }
            }
        }
        mesh_shader_->unuse(ctx);
    }

    if (!scene.nurbsnodes_.empty()) {
        // draw nurbs
        Pass::bind_inputs(nurbs_shader_->get_pre_shader(), eye, ctx);
        Pass::bind_inputs(*nurbs_shader_, eye, ctx);

        Pass::set_camera_matrices(nurbs_shader_->get_pre_shader(),
                                  camera,
                                  pipeline_->get_current_scene(eye),
                                  eye,
                                  ctx);
        Pass::set_camera_matrices(
            *nurbs_shader_, camera, pipeline_->get_current_scene(eye), eye, ctx);

        nurbs_shader_->set_material_uniforms(
            scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
        nurbs_shader_->set_material_uniforms(
            scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

        nurbs_shader_->set_uniform(ctx,
                                   pipeline_->config.get_max_tesselation(),
                                   "gua_max_tesselation");

        for (auto const& node : scene.nurbsnodes_)
            {
            auto geometry =
                GeometryDatabase::instance()->lookup(node.data.get_geometry());
            auto material =
                MaterialDatabase::instance()->lookup(node.data.get_material());

#ifdef DEBUG_XFB_OUTPUT
            scm::gl::transform_feedback_statistics_query_ptr q = ctx
                .render_device->create_transform_feedback_statistics_query(0);
            ctx.render_context->begin_query(q);
#endif
            // pre-tesselate if necessary
            nurbs_shader_->get_pre_shader().use(ctx);
            {
                nurbs_shader_->get_pre_shader()
                    .set_uniform(ctx, node.transform, "gua_model_matrix");
                nurbs_shader_->get_pre_shader().set_uniform(
                    ctx,
                    scm::math::transpose(scm::math::inverse(node.transform)),
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
                        ctx, node.transform, "gua_model_matrix");
                    nurbs_shader_->set_uniform(
                        ctx,
                        scm::math::transpose(
                            scm::math::inverse(node.transform)),
                        "gua_normal_matrix");

                    geometry->draw(ctx);
                }
            }
            nurbs_shader_->unuse(ctx);
        }
    }

#if 0
	if (!scene.volumenodes_.empty()) {

		mesh_shader_->set_material_uniforms(
			scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
		mesh_shader_->set_material_uniforms(
			scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

		Pass::bind_inputs(*volume_shader_, eye, ctx);
		Pass::set_camera_matrices(*volume_shader_,
			camera,
			pipeline_->get_current_scene(eye),
			eye,
			ctx);

		// draw volumes
		mesh_shader_->use(ctx);
		{
			for (auto const& node : scene.volumenodes_) {
				auto geometry =
					GeometryDatabase::instance()->lookup(node.data.get_geometry());
				auto material =
					MaterialDatabase::instance()->lookup(node.data.get_material());

				if (material && geometry) {
					mesh_shader_->set_uniform(
						ctx, material->get_id(), "gua_material_id");
					mesh_shader_->set_uniform(
						ctx, node.transform, "gua_model_matrix");
					mesh_shader_->set_uniform(
						ctx,
						scm::math::transpose(
						scm::math::inverse(node.transform)),
						"gua_normal_matrix");

					geometry->draw(ctx);
				}
			}
		}
		mesh_shader_->unuse(ctx);
	}
#else
	if (!scene.volumenodes_.empty()) {

		volume_shader_->set_material_uniforms(
			scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
		volume_shader_->set_material_uniforms(
			scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

		Pass::bind_inputs(*volume_shader_, eye, ctx);
		Pass::set_camera_matrices(*volume_shader_,
			camera,
			pipeline_->get_current_scene(eye),
			eye,
			ctx);

		// draw volumes
		volume_shader_->use(ctx);
		{
			for (auto const& node : scene.volumenodes_) {
				auto geometry =
					GeometryDatabase::instance()->lookup(node.data.get_geometry());
				auto material =
					MaterialDatabase::instance()->lookup(node.data.get_material());

				if (material && geometry) {
					volume_shader_->set_uniform(
						ctx, material->get_id(), "gua_material_id");
					volume_shader_->set_uniform(
						ctx, node.transform, "gua_model_matrix");
					volume_shader_->set_uniform(
						ctx,
						scm::math::transpose(
						scm::math::inverse(node.transform)),
						"gua_normal_matrix");

					geometry->draw(ctx);
				}
			}
		}
		volume_shader_->unuse(ctx);
	}
#endif


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
            auto geometry =
                GeometryDatabase::instance()->lookup(ray.data.get_geometry());

            mesh_shader_->set_uniform(
                ctx, bbox_material->get_id(), "gua_material_id");
            mesh_shader_->set_uniform(ctx, ray.transform, "gua_model_matrix");
            mesh_shader_->set_uniform(
                ctx,
                scm::math::transpose(scm::math::inverse(ray.transform)),
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
  volume_shader_->create(materials);
}


////////////////////////////////////////////////////////////////////////////////

LayerMapping const* GBufferPass::get_gbuffer_mapping() const {
  // todo: dirty to use single Ubershader mapping here -> possible solution:
  // extract gbuffermapping from Ubershader?
  return mesh_shader_->get_gbuffer_mapping();
}

////////////////////////////////////////////////////////////////////////////////

}
