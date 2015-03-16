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

#ifndef GUA_SHADOW_MAP_HPP
#define GUA_SHADOW_MAP_HPP

// guacamole headers
#include <gua/renderer/RenderTarget.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/utils/Mask.hpp>

namespace gua {

namespace node {
  class SpotLightNode;
}

class Pipeline;

/**
 *
 */
class ShadowMap /*: public RenderTarget*/ {
 public:

  struct CachedShadowMap {
    std::shared_ptr<Texture2D> shadow_map;
    Mask                       render_mask;
  };

  struct SharedResource {
    std::list<std::shared_ptr<Texture2D>>                     unused_shadow_maps;
    std::unordered_map<node::SpotLightNode*, CachedShadowMap> used_shadow_maps;
  };

  // ShadowMap();
  // virtual ~ShadowMap();

  math::vec2ui draw(Pipeline& pipe, node::SpotLightNode* light);

  void allocate(RenderContext& ctx);
  void clear_cache();

  // void render(Pipeline* pipe,
  //             math::mat4 const& transform,
  //             unsigned map_size);

  // void render_cascaded(RenderContext const& ctx,
  //             SceneGraph const& scene_graph,
  //             math::vec3 const& center_of_interest,
  //             Frustum const& scene_frustum,
  //             Camera const& scene_camera,
  //             math::mat4 const& transform,
  //             unsigned map_size,
  //             float split_0,
  //             float split_1,
  //             float split_2,
  //             float split_3,
  //             float split_4,
  //             float near_clipping_in_sun_direction);

  // ShadowMapBuffer*               get_buffer() const {return buffer_;}
  // std::vector<math::mat4> const& get_projection_view_matrices() const {return projection_view_matrices_;}

  // virtual void cleanup(RenderContext const& context);

  // inline std::size_t const uuid() const { return reinterpret_cast<std::size_t>(buffer_); }

 private:

  std::shared_ptr<SharedResource> res_ = nullptr;

  // void update_members(RenderContext const& ctx, unsigned map_size);
  // void render_geometry(
  //   Pipeline* pipe, Frustum const& shadow_frustum,
  //   unsigned cascade, unsigned map_size
  // );

  // std::unique_ptr<Serializer> serializer_;

  // ShadowMapBuffer* buffer_;

  // scm::gl::depth_stencil_state_ptr          depth_stencil_state_;
  // scm::gl::rasterizer_state_ptr             rasterizer_state_;
  // std::vector<math::mat4>                   projection_view_matrices_;
  // std::shared_ptr<gua::CameraUniformBlock>  camera_block_;
};

}

#endif  // GUA_SHADOW_MAP_HPP
