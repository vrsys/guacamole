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
#include <gua/renderer/TriMeshUberShader.hpp>
#include <gua/renderer/NURBSUberShader.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/memory.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

ShadowMap::ShadowMap(Pipeline* pipeline)
    : serializer_(gua::make_unique<Serializer>()),
      pipeline_(pipeline),
      buffer_(nullptr),
      projection_view_matrices_() {
}


////////////////////////////////////////////////////////////////////////////////

ShadowMap::~ShadowMap() {
    if (buffer_)
      delete buffer_;
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

void ShadowMap::cleanup(RenderContext const& context)
{
  if (buffer_) buffer_->remove_buffers(context);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::render_geometry(RenderContext const & ctx,
                                SceneGraph const& current_graph,
                                math::vec3 const& center_of_interest,
                                Frustum const& shadow_frustum,
                                Camera const& scene_camera,
                                unsigned cascade,
                                unsigned map_size) {
  SerializedScene scene;
  scene.frustum = shadow_frustum;
  scene.center_of_interest = center_of_interest;
  scene.enable_global_clipping_plane = pipeline_->config.get_enable_global_clipping_plane();
  scene.global_clipping_plane = pipeline_->config.get_global_clipping_plane();
  serializer_->check(&scene,
                     &current_graph,
                     scene_camera.render_mask,
                     false,
                     false,
                     true);

  projection_view_matrices_[cascade] =
      shadow_frustum.get_projection() * shadow_frustum.get_view();

  for (auto const& type : scene.geometrynodes_)
  {
    // pointer to appropriate ubershader
    GeometryUberShader* ubershader = nullptr;

    // get appropriate ubershader
    if (!type.second.empty())
    {
      auto const& filename = type.second.front()->get_filename();
      auto geometry = GeometryDatabase::instance()->lookup(filename);

      if (geometry) {
        ubershader = pipeline_->get_geometry_ubershaders().at(type.first).get();
      } else {
        Logger::LOG_WARNING << "ShadowMap::render_geometry(): No such file/geometry " << filename << std::endl;
      }
    }

    if (ubershader)

    {
      auto camera_position(scene.frustum.get_camera_position());
      auto projection(scene.frustum.get_projection());
      auto view_matrix(scene.frustum.get_view());

      ubershader->set_uniform(ctx, camera_position, "gua_camera_position");
      ubershader->set_uniform(ctx, projection, "gua_projection_matrix");
      ubershader->set_uniform(ctx, view_matrix, "gua_view_matrix");
      ubershader->set_uniform(ctx, scm::math::inverse(projection * view_matrix), "gua_inverse_projection_view_matrix");

      ubershader->set_uniform(ctx, scene.enable_global_clipping_plane, "gua_enable_global_clipping_plane");
      ubershader->set_uniform(ctx, scene.global_clipping_plane, "gua_global_clipping_plane");

      ubershader->set_uniform(ctx, 1.0f / map_size, "gua_texel_width");
      ubershader->set_uniform(ctx, 1.0f / map_size, "gua_texel_height");
      ubershader->set_uniform(ctx, true, "gua_render_shadow_map");

      for (auto const& node : type.second)
      {
        if (node->get_shadow_mode() != ShadowMode::OFF) {
          ubershader->set_uniform(ctx, static_cast<int>(node->get_shadow_mode()), "gua_shadow_quality");
        }

        // draw node
        ubershader->draw(ctx,
          node->get_filename(),
          node->get_material(),
          node->get_cached_world_transform(),
          scm::math::transpose(scm::math::inverse(node->get_cached_world_transform())),
          scene.frustum,
          uuid());
      }
    } else {
      Logger::LOG_WARNING << "ShadowMap::render_geometry(): UberShader missing." << std::endl;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::render(RenderContext const& ctx,
                       SceneGraph const& scene_graph,
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

    // render geometries
    render_geometry(ctx, scene_graph, center_of_interest, shadow_frustum, scene_camera, 0, map_size);

    ctx.render_context->reset_state_objects();

    buffer_->unbind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void ShadowMap::render_cascaded(RenderContext const& ctx,
              SceneGraph const& scene_graph,
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

  for (int y(0); y<2; ++y) {
    for (int x(0); x<2; ++x) {

      int cascade(y*2 + x);

      // render each cascade to a quarter of the shadow map
      ctx.render_context->set_viewport(scm::gl::viewport(
          math::vec2(x * map_size, y * map_size),
          math::vec2(map_size, map_size)));

      // set clipping of camera frustum according to current cascade
      Frustum cropped_frustum(Frustum::perspective(
        scene_frustum.get_camera_transform(),
        scene_frustum.get_screen_transform(),
        splits[cascade], splits[cascade+1]
      ));

      // transform cropped frustum tu sun space and calculate radius and bbox
      // of transformed frustum
      auto cropped_frustum_corners(cropped_frustum.get_corners());
      math::BoundingBox<math::vec3> extends_in_sun_space;
      float radius_in_sun_space = 0;
      std::vector<math::vec3> corners_in_sun_space;
      math::vec3 center_in_sun_space(0, 0, 0);

      auto inverse_sun_transform(scm::math::inverse(transform));
      for (auto const& corner: cropped_frustum_corners) {
        math::vec3 new_corner(inverse_sun_transform * corner);
        center_in_sun_space += new_corner/8;
        corners_in_sun_space.push_back(new_corner);
        extends_in_sun_space.expandBy(new_corner);
      }

      for (auto const& corner: corners_in_sun_space) {
        float radius = scm::math::length(corner-center_in_sun_space);
        if (radius > radius_in_sun_space)
          radius_in_sun_space = radius;
      }

      // center of front plane of frustum
      auto center(math::vec3((extends_in_sun_space.min[0] + extends_in_sun_space.max[0])/2,
                             (extends_in_sun_space.min[1] + extends_in_sun_space.max[1])/2,
                              extends_in_sun_space.max[2] + near_clipping_in_sun_direction));

      // eliminate sub-pixel movement
      float tex_coord_x = center.x * map_size / radius_in_sun_space / 2;
      float tex_coord_y = center.y * map_size / radius_in_sun_space / 2;

      float tex_coord_rounded_x = round(tex_coord_x);
      float tex_coord_rounded_y = round(tex_coord_y);

      float dx = tex_coord_rounded_x - tex_coord_x;
      float dy = tex_coord_rounded_y - tex_coord_y;

      dx /= map_size / radius_in_sun_space / 2;
      dy /= map_size / radius_in_sun_space / 2;

      center.x += dx;
      center.y += dy;

      // calculate transformation of shadow screen
      auto screen_in_sun_space(scm::math::make_translation(center) * scm::math::make_scale(radius_in_sun_space*2, radius_in_sun_space*2, 1.0f));
      auto sun_screen_transform(transform * screen_in_sun_space);

      // calculate transformation of shadow eye
      auto sun_eye_transform(scm::math::make_translation(sun_screen_transform.column(3)[0], sun_screen_transform.column(3)[1], sun_screen_transform.column(3)[2]));
      auto sun_eye_depth(transform * math::vec4(0, 0, extends_in_sun_space.max[2] - extends_in_sun_space.min[2] + near_clipping_in_sun_direction, 0.0f));

      auto shadow_frustum(
        Frustum::orthographic(
          sun_eye_transform,
          sun_screen_transform,
          0,
          scm::math::length(sun_eye_depth)
        )
      );

      // render geometries
      render_geometry(ctx, scene_graph, center_of_interest, shadow_frustum, scene_camera, cascade, map_size);
    }
  }

  ctx.render_context->reset_state_objects();

  buffer_->unbind(ctx);

}

////////////////////////////////////////////////////////////////////////////////

}
