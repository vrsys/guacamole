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
#include <gua/math.hpp>
#include <gua/renderer/RenderContext.hpp>

#include <unordered_map>
#include <typeindex>

namespace gua {

struct Camera;
class Serializer;
class GBuffer;
class Frustum;
class Pipeline;
class GeometryUberShader;


class SceneGraph;

/**
 *
 */
class ShadowMap {
 public:

  /**
   *
   */
  ShadowMap(Pipeline* pipeline);

  virtual ~ShadowMap();

  void render(RenderContext const& ctx,
              SceneGraph const& current_graph,
              math::vec3 const& center_of_interest,
              Camera const& scene_camera,
              math::mat4 const& transform,
              unsigned map_size);

  void render_cascaded(RenderContext const& ctx,
              SceneGraph const& scene_graph,
              math::vec3 const& center_of_interest,
              Frustum const& scene_frustum,
              Camera const& scene_camera,
              math::mat4 const& transform,
              unsigned map_size,
              float split_0,
              float split_1,
              float split_2,
              float split_3,
              float split_4,
              float near_clipping_in_sun_direction);

  bool pre_compile_shaders(RenderContext const& ctx);

  GBuffer*                       get_buffer() const {return buffer_;}
  std::vector<math::mat4> const& get_projection_view_matrices() const {return projection_view_matrices_;}

  virtual void cleanup(RenderContext const& context);

 private:

  void update_members(RenderContext const& ctx, unsigned map_size);
  void render_geometry(RenderContext const & ctx,
                       SceneGraph const& scene_graph,
                       math::vec3 const& center_of_interest,
                       Frustum const& shadow_frustum,
                       Camera const& scene_camera,
                       unsigned cascade,
                       unsigned map_size);

  std::unique_ptr<Serializer> serializer_;

  GBuffer* buffer_;
  Pipeline* pipeline_;

  std::unordered_map<std::type_index, GeometryUberShader*> ubershader_;
  
  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
  scm::gl::rasterizer_state_ptr rasterizer_state_;
  std::vector<math::mat4> projection_view_matrices_;
};

}

#endif  // GUA_SHADOW_MAP_HPP
