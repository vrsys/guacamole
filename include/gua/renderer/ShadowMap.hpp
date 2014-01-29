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

namespace gua {

class Serializer;
class GBuffer;
class Frustum;
class Camera;
class Pipeline;
class ShadowMapMeshShader;
class ShadowMapNURBSShader;

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
              Camera const& scene_camera,
              math::mat4 const& transform,
              unsigned map_size);

  void render_cascaded(RenderContext const& ctx,
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

  void print_shaders(std::string const& directory,
                     std::string const& name) const;

  bool pre_compile_shaders(RenderContext const& ctx);

  GBuffer*                       get_buffer() const {return buffer_;}
  std::vector<math::mat4> const& get_projection_view_matrices() const {return projection_view_matrices_;}

 private:

  void update_members(RenderContext const& ctx, unsigned map_size);
  void render_geometry(RenderContext const & ctx, Frustum const& shadow_frustum, Camera const& scene_camera, unsigned cascade);

  GBuffer* buffer_;

  Serializer* serializer_;

  Pipeline* pipeline_;

  ShadowMapMeshShader* mesh_shader_;
  ShadowMapNURBSShader* nurbs_shader_;

  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
  scm::gl::rasterizer_state_ptr rasterizer_state_;
  std::vector<math::mat4> projection_view_matrices_;
};

}

#endif  // GUA_SHADOW_MAP_HPP
