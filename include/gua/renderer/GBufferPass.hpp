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

#ifndef GUA_GBUFFER_PASS_HPP
#define GUA_GBUFFER_PASS_HPP

// guacamole headers
#include <gua/renderer/GeometryPass.hpp>
#include <gua/renderer/Mesh.hpp>

namespace gua {

class Pipeline;
class GBufferMeshUberShader;
class GBufferNURBSUberShader;
class GBufferVideo3DUberShader;

/**
 *
 */
class GBufferPass : public GeometryPass {
 public:

  /**
   *
   */
  GBufferPass(Pipeline* pipeline);

  /**
   * Destructor.
   *
   * Deletes the FullscreenPass and frees all associated data.
   */
  virtual ~GBufferPass();

  void create(
      RenderContext const& ctx,
      std::vector<std::pair<BufferComponent,
                            scm::gl::sampler_state_desc> > const& layers);

  /*virtual*/ void print_shaders(std::string const& directory,
                     std::string const& name) const;

  bool pre_compile_shaders(RenderContext const& ctx);

  void apply_material_mapping(std::set<std::string> const& materials) const;

  LayerMapping const* get_gbuffer_mapping() const;

 private:

  void rendering(SerializedScene const& scene,
                 RenderContext const& ctx,
                 CameraMode eye,
                 Camera const& camera,
                 FrameBufferObject* target);

  GBufferMeshUberShader* mesh_shader_;
  GBufferNURBSUberShader* nurbs_shader_;
  GBufferVideo3DUberShader* video3D_shader_;

  /**
   * Ugly hack! "bfc" means backface culling.
   * TODO: implement per material culling
   */
  scm::gl::rasterizer_state_ptr bfc_rasterizer_state_;
  scm::gl::rasterizer_state_ptr no_bfc_rasterizer_state_;

  scm::gl::rasterizer_state_ptr bbox_rasterizer_state_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_;

  std::shared_ptr<Geometry> bounding_box_;
};

}

#endif  // GUA_GBUFFER_PASS_HPP
