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
#include <gua/renderer/GeometryRessource.hpp>

#include <unordered_map>

namespace gua {

class Pipeline;
class SceneGraph;
class TriMeshUberShader;
class GeometryUberShader;

/**
 *
 */
class GBufferPass : public GeometryPass {
 public:

   typedef std::unordered_map<std::type_index, std::shared_ptr<GeometryUberShader>> GeometryUberShaderMap;

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
                        scm::gl::sampler_state_desc> > const& layers) override;

  void cleanup(RenderContext const& ctx) override;

  bool pre_compile_shaders(const gua::RenderContext &) override;

  void print_shaders(std::string const& directory,
                     std::string const& name) const override;

  void apply_material_mapping(std::set<std::string> const& materials);

  LayerMapping const* get_gbuffer_mapping() const;

  inline GeometryUberShaderMap const& get_geometry_ubershaders() const { return ubershaders_; }

 private: // methods

  void rendering(SerializedScene const& scene,
                 SceneGraph const&,
                 RenderContext const& ctx,
                 CameraMode eye,
                 Camera const& camera,
                 FrameBufferObject* target,
                 View const& view) override;

  void display_bboxes(RenderContext const& ctx,
                      SerializedScene const& scene,
                      View const& view);
  void display_rays  (RenderContext const& ctx,
                      SerializedScene const& scene,
                      View const& view);
  void display_quads (RenderContext const& ctx,
                      SerializedScene const& scene,
                      CameraMode eye,
                      View const& view);

  void update_ubershader_from_scene(RenderContext const& ctx,
                                    SerializedScene const& scene,
                                    SceneGraph const& graph);

  void initialize_state_objects(RenderContext const& ctx);

  void initialize_trimesh_ubershader(RenderContext const& ctx) const;

  private: // attributes

  /**
  * all ubershaders used in scene
  */
  mutable GeometryUberShaderMap ubershaders_;

  /**
  * copy of all material names in scene
  *
  *  - necessary to generate gbuffermappings of ubershaders
  */
  std::set<std::string> cached_materials_;

  /**
   * Ugly hack! "bfc" means backface culling.
   * TODO: implement per material culling
   */
  scm::gl::rasterizer_state_ptr bfc_rasterizer_state_;
  scm::gl::rasterizer_state_ptr no_bfc_rasterizer_state_;

  scm::gl::rasterizer_state_ptr bbox_rasterizer_state_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
};

}

#endif  // GUA_GBUFFER_PASS_HPP
