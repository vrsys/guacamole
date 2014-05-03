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
#include <algorithm>

#include <gua/platform.hpp>
#include <gua/utils.hpp>
#include <gua/traverse.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/TriMeshUberShader.hpp>
#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/GeometryUberShader.hpp>

#include <gua/scenegraph/GeometryNode.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

#include <gua/databases.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////

  GBufferPass::GBufferPass(Pipeline* pipeline)
    : GeometryPass(pipeline),
    bfc_rasterizer_state_(),
    no_bfc_rasterizer_state_(),
    bbox_rasterizer_state_(),
    depth_stencil_state_()
  {}

  ////////////////////////////////////////////////////////////////////////////////

  GBufferPass::~GBufferPass()
  {}

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

  void GBufferPass::rendering(SerializedScene const& scene,
    SceneGraph const& graph,
    RenderContext const& ctx,
    CameraMode eye,
    Camera const& camera,
    FrameBufferObject* target) {

    if (!depth_stencil_state_ ||
        !bfc_rasterizer_state_ ||
        !no_bfc_rasterizer_state_ ||
        !no_bfc_rasterizer_state_) 
    {
      initialize_state_objects(ctx);
    }

    ctx.render_context->set_rasterizer_state(
      pipeline_->config.enable_backface_culling() ? bfc_rasterizer_state_
      : no_bfc_rasterizer_state_);

    ctx.render_context->set_depth_stencil_state(depth_stencil_state_);

    // make sure all ubershaders are available
    update_ubershader_from_scene(scene, graph);

    // draw all drawable geometries
    for (auto const& type_ressource_pair : scene.geometrynodes_)
    {
      auto const& type = type_ressource_pair.first;
      auto const& ressource_container = type_ressource_pair.second;
      auto ubershader = ubershaders_.at(type);

      // set frame-consistent per-ubershader uniforms
      ubershader->set_material_uniforms(scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
      ubershader->set_material_uniforms(scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

      ubershader->set_uniform(ctx, scene.enable_global_clipping_plane, "gua_enable_global_clipping_plane");
      ubershader->set_uniform(ctx, scene.global_clipping_plane, "gua_global_clipping_plane");

      for (auto const& program : ubershader->programs())
      {
        Pass::bind_inputs(*program, eye, ctx);
        Pass::set_camera_matrices(*program,
          camera,
          pipeline_->get_current_scene(eye),
          eye,
          ctx);
      }

      // 1. call preframe callback if available for type
      if (ubershader->get_stage_mask() & GeometryUberShader::PRE_FRAME_STAGE) 
      {
        ubershader->preframe(ctx);
      }

      // 2. iterate all drawables of current type and call predraw of current ubershader
      if (ubershader->get_stage_mask() & GeometryUberShader::PRE_DRAW_STAGE) 
      {
        for (auto const& node : ressource_container)
        {
          auto const& ressource = GeometryDatabase::instance()->lookup(node->get_filename());
          auto const& material = MaterialDatabase::instance()->lookup(node->get_material());

          if (ressource && material)
          {
            ubershader->predraw(ctx,
              node->get_filename(),
              node->get_material(),
              node->get_cached_world_transform(),
              scm::math::transpose(scm::math::inverse(node->get_cached_world_transform())),
              scene.frustum);
          }
        }
      }

      // 3. iterate all drawables of current type and call draw of current ubershader
      if (ubershader->get_stage_mask() & GeometryUberShader::DRAW_STAGE)
      {
        for (auto const& node : ressource_container)
        {
          auto const& ressource = GeometryDatabase::instance()->lookup(node->get_filename());
          auto const& material = MaterialDatabase::instance()->lookup(node->get_material());

          if (ressource && material)
          {
            ubershader->draw(ctx,
              node->get_filename(),
              node->get_material(),
              node->get_cached_world_transform(),
              scm::math::transpose(scm::math::inverse(node->get_cached_world_transform())),
              scene.frustum);
          }
        }
      }

      // 4. iterate all drawables of current type and call postdraw of current ubershader
      if (ubershader->get_stage_mask() & GeometryUberShader::POST_DRAW_STAGE)
      {
        for (auto const& node : ressource_container)
        {
          auto const& ressource = GeometryDatabase::instance()->lookup(node->get_filename());
          auto const& material = MaterialDatabase::instance()->lookup(node->get_material());

          if (ressource && material)
          {
            ubershader->postdraw(ctx,
              node->get_filename(),
              node->get_material(),
              node->get_cached_world_transform(),
              scm::math::transpose(scm::math::inverse(node->get_cached_world_transform())),
              scene.frustum);
          }
        }
      }

      // 5. call postframe callback if available for type
      if (ubershader->get_stage_mask() & GeometryUberShader::POST_FRAME_STAGE) 
      {
        ubershader->postframe(ctx);
      }
    }

    ///////////////////////////////////////////////////////////////
    // draw debug and helper information
    ///////////////////////////////////////////////////////////////
    display_quads(ctx, scene, eye);

    ctx.render_context->set_rasterizer_state(bbox_rasterizer_state_);
    display_bboxes(ctx, scene);
    display_rays(ctx, scene);

    ctx.render_context->reset_state_objects();
  }

  ////////////////////////////////////////////////////////////////////////////////

  void GBufferPass::display_bboxes(RenderContext const& ctx, SerializedScene const& scene)
  {
    auto meshubershader = Singleton<TriMeshUberShader>::instance();

    if (pipeline_->config.enable_bbox_display())
    {
      meshubershader->get_program()->use(ctx);
      for (auto const& bbox : scene.bounding_boxes_)
      {
        math::mat4 bbox_transform(math::mat4::identity());

        auto scale(scm::math::make_scale((bbox.max - bbox.min) * 1.001f));
        auto translation(scm::math::make_translation((bbox.max + bbox.min) / 2.f));

        bbox_transform *= translation;
        bbox_transform *= scale;

        meshubershader->draw(ctx,
          "gua_bounding_box_geometry",
          "gua_bounding_box",
          bbox_transform,
          scm::math::transpose(scm::math::inverse(bbox_transform)),
          scene.frustum);
      }
      meshubershader->get_program()->unuse(ctx);
    }
  }


  ////////////////////////////////////////////////////////////////////////////////

  void GBufferPass::display_rays(RenderContext const& ctx, SerializedScene const& scene)
  {
    auto meshubershader = Singleton<TriMeshUberShader>::instance();  

    if (pipeline_->config.enable_ray_display())
    {
      meshubershader->get_program()->use(ctx);
      for (auto const& ray : scene.rays_)
      {
        meshubershader->draw(ctx,
          "gua_plane_geometry",
          "gua_bounding_box",
          ray->get_cached_world_transform(),
          scm::math::inverse(ray->get_cached_world_transform()),
          scene.frustum);
      }
      meshubershader->get_program()->unuse(ctx);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////

  void GBufferPass::display_quads(RenderContext const& ctx, SerializedScene const& scene, CameraMode eye)
  {
    auto meshubershader = Singleton<TriMeshUberShader>::instance();

    if (!scene.textured_quads_.empty()) {
      meshubershader->get_program()->use(ctx);
      {
        for (auto const& node : scene.textured_quads_)
        {
          std::string texture_name(node->data.get_texture());
          if (node->data.get_is_stereo_texture()) {

            if (eye == CameraMode::LEFT) {
              texture_name += "_left";
            }
            else if (eye == CameraMode::RIGHT) {
              texture_name += "_right";
            }
          }

          if (TextureDatabase::instance()->is_supported(texture_name)) {
            auto texture = TextureDatabase::instance()->lookup(texture_name);
            auto mapped_texture(meshubershader->get_uniform_mapping()->get_mapping("gua_textured_quad", "texture"));

            meshubershader->set_uniform(ctx, texture, mapped_texture.first, mapped_texture.second);

            auto mapped_flip_x(meshubershader->get_uniform_mapping()->get_mapping("gua_textured_quad", "flip_x"));
            meshubershader->set_uniform(ctx, node->data.get_flip_x(), mapped_flip_x.first, mapped_flip_x.second);

            auto mapped_flip_y(meshubershader->get_uniform_mapping()->get_mapping("gua_textured_quad", "flip_y"));
            meshubershader->set_uniform(ctx, node->data.get_flip_y(), mapped_flip_y.first, mapped_flip_y.second);
          }

          meshubershader->draw(ctx,
            "gua_plane_geometry",
            "gua_textured_quad",
            node->get_scaled_world_transform(),
            scm::math::inverse(node->get_scaled_world_transform()),
            scene.frustum);
        }
      }
      meshubershader->get_program()->unuse(ctx);
    }
  }


  ////////////////////////////////////////////////////////////////////////////////

  void GBufferPass::update_ubershader_from_scene(SerializedScene const& scene, SceneGraph const& graph)
  {
    bool ubershader_available = true;
    for (auto const& geometry_pair : scene.geometrynodes_)
    {
      ubershader_available = ubershader_available && ubershaders_.count(geometry_pair.first);
    }

    if (!ubershader_available)
    {
      auto get_ubershader = [&] (Node* n) { 
        GeometryNode* geode = dynamic_cast<GeometryNode*>(n);
        if (geode) {
          std::type_index type(typeid(*geode));
          if (!ubershaders_.count(type)) {
            auto const& ressource = GeometryDatabase::instance()->lookup(geode->get_filename());
            if (ressource) {
              auto ubershader = ressource->get_ubershader();
              ubershader->create(materials_);
              ubershaders_[type] = ubershader;
            }
          }
        }
      };
      gua::dfs_traverse(graph.get_root().get(), get_ubershader);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////

  void GBufferPass::initialize_state_objects(RenderContext const& ctx)
  {
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
  }


  ////////////////////////////////////////////////////////////////////////////////

  void GBufferPass::apply_material_mapping(std::set<std::string> const& materials) 
  {
    materials_ = materials;
    Singleton<TriMeshUberShader>::instance()->create(materials_);
  }


  ////////////////////////////////////////////////////////////////////////////////

  LayerMapping const* GBufferPass::get_gbuffer_mapping() const {
    return Singleton<TriMeshUberShader>::instance()->get_gbuffer_mapping();
  }


}
