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
#include <gua/renderer/ShadowMap.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Serializer.hpp>
#include <gua/renderer/GBufferMeshUberShader.hpp>
#include <gua/renderer/GBufferNURBSUberShader.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

ShadowMap::ShadowMap(Pipeline* pipeline)
    : serializer_(new Serializer),
      pipeline_(pipeline),
      mesh_shader_(new GBufferMeshUberShader),
      // nurbs_shader_(new GBufferNURBSUberShader),
      buffer_(nullptr),
      projection_view_matrices_() {
}


////////////////////////////////////////////////////////////////////////////////

ShadowMap::~ShadowMap() {
    if (mesh_shader_)
      delete mesh_shader_;
    // if (nurbs_shader_)
    //   delete nurbs_shader_;
    if (serializer_)
      delete serializer_;
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::print_shaders(std::string const& directory,
                              std::string const& name) const {
  mesh_shader_->get_pass(0)->save_to_file(directory, name + "/shadow/mesh");
  // nurbs_shader_->save_to_file(directory, name + "/shadow/nurbs");
}

////////////////////////////////////////////////////////////////////////////////

bool ShadowMap::pre_compile_shaders(RenderContext const& ctx) {

    bool success(false);

    if (mesh_shader_)             success = mesh_shader_->upload_to(ctx);
    // if (success && nurbs_shader_) success = nurbs_shader_->upload_to(ctx);

    return success;
}


////////////////////////////////////////////////////////////////////////////////

void ShadowMap::update_members(RenderContext const & ctx, unsigned map_size) {
    //check whether shadow map size is sufficient
    if (buffer_ && buffer_->width() < map_size) {
      buffer_->remove_buffers(ctx);
      delete buffer_;
      buffer_ = nullptr;
    }

    if (!buffer_) {
        scm::gl::sampler_state_desc state;
        state._compare_mode = scm::gl::TEXCOMPARE_COMPARE_REF_TO_TEXTURE;

        buffer_ = new GBuffer({{ BufferComponent::DEPTH_16, state }}, map_size, map_size);
        buffer_->create(ctx);
    }

    // let derived class render all geometries
    if (!depth_stencil_state_)
        depth_stencil_state_ =
            ctx.render_device->create_depth_stencil_state(true, true);

    if (!rasterizer_state_)
        rasterizer_state_ = ctx.render_device
            ->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::apply_material_mapping(std::set<std::string> const &
                                         materials) const {
  mesh_shader_->create(materials);
  // nurbs_shader_->create(materials);
}


////////////////////////////////////////////////////////////////////////////////

void ShadowMap::render_geometry(RenderContext const & ctx,
                                math::vec3 const& center_of_interest,
                                Frustum const& shadow_frustum,
                                Camera const& scene_camera,
                                unsigned cascade) {
  SerializedScene scene;
  scene.frustum = shadow_frustum;
  scene.center_of_interest = center_of_interest;
  scene.enable_global_clipping_plane = pipeline_->config.get_enable_global_clipping_plane();
  scene.global_clipping_plane = pipeline_->config.get_global_clipping_plane();
  serializer_->check(&scene,
                     pipeline_->get_current_graph(),
                     scene_camera.render_mask,
                     false,
                     false,
                     true);

  projection_view_matrices_[cascade] =
      shadow_frustum.get_projection() * shadow_frustum.get_view();

  mesh_shader_->set_material_uniforms(
      scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
  mesh_shader_->set_material_uniforms(
      scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

  mesh_shader_->set_uniform(ctx, scene.enable_global_clipping_plane, "gua_enable_global_clipping_plane");
  mesh_shader_->set_uniform(ctx, scene.global_clipping_plane, "gua_global_clipping_plane");

  auto camera_position(scene.frustum.get_camera_position());
  auto projection(scene.frustum.get_projection());
  auto view_matrix(scene.frustum.get_view());

  mesh_shader_->set_uniform(ctx, camera_position, "gua_camera_position");
  mesh_shader_->set_uniform(ctx, projection, "gua_projection_matrix");
  mesh_shader_->set_uniform(ctx, view_matrix, "gua_view_matrix");
  mesh_shader_->set_uniform(ctx, scm::math::inverse(projection * view_matrix), "gua_inverse_projection_view_matrix");

  for (auto const& node : scene.meshnodes_) {
      auto geometry = GeometryDatabase::instance()->lookup(node->get_geometry());
      auto material = MaterialDatabase::instance()->lookup(node->get_material());
      if (geometry) {
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

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::render(RenderContext const& ctx,
                       math::vec3 const& center_of_interest,
                       Camera const& scene_camera,
                       math::mat4 const& transform,
                       unsigned map_size) {

    // init members
    update_members(ctx, map_size);
    projection_view_matrices_ = std::vector<math::mat4>(1);

    buffer_->bind(ctx);
    buffer_->clear_depth_stencil_buffer(ctx);

    ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
    ctx.render_context->set_rasterizer_state(rasterizer_state_);


    ctx.render_context->set_viewport(scm::gl::viewport(
        math::vec2(0, 0),
        math::vec2(map_size, map_size)));

    // calculate light frustum
    math::mat4 screen_transform(scm::math::make_translation(0.f, 0.f, -1.f));
    screen_transform = transform * screen_transform;

    Frustum shadow_frustum = Frustum::perspective(transform,
                           screen_transform,
                           pipeline_->config.near_clip(),
                           pipeline_->config.far_clip());

    mesh_shader_->set_uniform(ctx, 1.0f / map_size, "gua_texel_width");
    mesh_shader_->set_uniform(ctx, 1.0f / map_size, "gua_texel_height");

    // render geometries
    mesh_shader_->get_pass(0)->use(ctx);
    render_geometry(ctx, center_of_interest, shadow_frustum, scene_camera, 0);
    mesh_shader_->get_pass(0)->unuse(ctx);

    ctx.render_context->reset_state_objects();

    buffer_->unbind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::render_cascaded(RenderContext const& ctx,
              math::vec3 const& center_of_interest,
              Frustum const& scene_frustum,
              Camera const& scene_camera,
              math::mat4 const& transform,
              unsigned map_size, float split_0,
              float split_1, float split_2,
              float split_3, float split_4,
              float near_clipping_in_sun_direction) {

  update_members(ctx, map_size*2);
  projection_view_matrices_ = std::vector<math::mat4>(4);

  buffer_->bind(ctx);
  buffer_->clear_depth_stencil_buffer(ctx);

  ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
  ctx.render_context->set_rasterizer_state(rasterizer_state_);

  std::vector<float> splits({
    split_0, split_1, split_2, split_3, split_4
  });

  if (pipeline_->config.near_clip() > split_0 || pipeline_->config.far_clip() < split_4) {
    Logger::LOG_WARNING << "Splits of cascaded shadow maps are not inside clipping range! Fallback to equidistant splits used." << std::endl;
    float clipping_range(pipeline_->config.far_clip() - pipeline_->config.near_clip());
    splits = {
      pipeline_->config.near_clip(),
      pipeline_->config.near_clip() + clipping_range * 0.25f,
      pipeline_->config.near_clip() + clipping_range * 0.5f,
      pipeline_->config.near_clip() + clipping_range * 0.75f,
      pipeline_->config.far_clip()
    };
  }

  mesh_shader_->set_uniform(ctx, 1.0f / map_size, "gua_texel_width");
  mesh_shader_->set_uniform(ctx, 1.0f / map_size, "gua_texel_height");

  for (int y(0); y<2; ++y) {
    for (int x(0); x<2; ++x) {

      int cascade(y*2 + x);

      ctx.render_context->set_viewport(scm::gl::viewport(
          math::vec2(x * map_size, y * map_size),
          math::vec2(map_size, map_size)));

      Frustum cropped_frustum(Frustum::perspective(
        scene_frustum.get_camera_transform(),
        scene_frustum.get_screen_transform(),
        splits[cascade], splits[cascade+1]
      ));

      auto cropped_frustum_corners(cropped_frustum.get_corners());
      math::BoundingBox<math::vec3> extends_in_sun_space;

      auto inverse_sun_transform(scm::math::inverse(transform));
      for (auto const& corner: cropped_frustum_corners) {
        auto corner_in_sun_space(inverse_sun_transform * corner);
        extends_in_sun_space.expandBy(corner_in_sun_space);
      }

      auto size(extends_in_sun_space.max - extends_in_sun_space.min);

      auto center(math::vec3((extends_in_sun_space.min[0] + extends_in_sun_space.max[0])/2,
                             (extends_in_sun_space.min[1] + extends_in_sun_space.max[1])/2,
                              extends_in_sun_space.max[2] + near_clipping_in_sun_direction));

      auto screen_in_sun_space(scm::math::make_translation(center) * scm::math::make_scale(size[0], size[1], 1.0f));

      auto sun_screen_transform(transform * screen_in_sun_space);
      auto sun_camera_transform(scm::math::make_translation(sun_screen_transform.column(3)[0], sun_screen_transform.column(3)[1], sun_screen_transform.column(3)[2]));
      auto sun_camera_depth(transform * math::vec4(0, 0, size[2] + near_clipping_in_sun_direction, 0.0f));

      auto shadow_frustum(
        Frustum::orthographic(
          sun_camera_transform,
          sun_screen_transform,
          0,
          scm::math::length(sun_camera_depth)
        )
      );

      // // render geometries
      mesh_shader_->get_pass(0)->use(ctx);
      render_geometry(ctx, center_of_interest, shadow_frustum, scene_camera, cascade);
      mesh_shader_->get_pass(0)->unuse(ctx);
    }
  }

  ctx.render_context->reset_state_objects();

  buffer_->unbind(ctx);

}

////////////////////////////////////////////////////////////////////////////////

}
