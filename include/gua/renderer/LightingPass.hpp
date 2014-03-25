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

#ifndef GUA_LIGHTING_PASS_HPP
#define GUA_LIGHTING_PASS_HPP

// guacamole headers
#include <gua/renderer/GeometryPass.hpp>
#include <gua/renderer/ShadowMap.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Geometry.hpp>

namespace gua {

class LightingUberShader;
class Serializer;
class LayerMapping;

/**
 *
 */
class LightingPass : public GeometryPass {
 public:

  /**
   *
   */
  LightingPass(Pipeline* pipeline);

  /**
   * Destructor.
   *
   * Deletes the FullscreenPass and frees all associated data.
   */
  virtual ~LightingPass();

  void apply_material_mapping(
      std::set<std::string> const& material_names,
      std::vector<LayerMapping const*> const& inputs) const;

  LayerMapping const* get_gbuffer_mapping() const;

  /* virtual */ void print_shaders(std::string const& directory,
                                   std::string const& name) const;

  bool pre_compile_shaders(RenderContext const& ctx);

public:
  ShadowMap shadow_map_;

 private:
  void rendering(SerializedScene const& scene,
                 SceneGraph const& scene_graph,
                 RenderContext const& ctx,
                 CameraMode eye,
                 Camera const& camera,
                 FrameBufferObject* target);

  void init_resources(RenderContext const& ctx);

  LightingUberShader* shader_;
  std::shared_ptr<Geometry> light_sphere_;
  std::shared_ptr<Geometry> light_cone_;
  scm::gl::quad_geometry_ptr fullscreen_quad_;

  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
  scm::gl::rasterizer_state_ptr rasterizer_state_front_;
  scm::gl::rasterizer_state_ptr rasterizer_state_back_;
  scm::gl::blend_state_ptr blend_state_;
};

}

#endif  // GUA_LIGHTING_PASS_HPP
