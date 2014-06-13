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

#ifndef GUA_FINAL_PASS_HPP
#define GUA_FINAL_PASS_HPP

// guacamole headers
#include <memory>
#include <gua/renderer/FullscreenPass.hpp>

namespace gua {

class Pipeline;
class FinalUberShader;
class LayerMapping;

/**
 *
 */
class FinalPass : public FullscreenPass {
 public:

  /**
   *
   */
  FinalPass(Pipeline* pipeline);

  /**
   *
   */
  void print_shaders(std::string const& directory,
                     std::string const& name) const;

  bool pre_compile_shaders(RenderContext const& ctx);

  void pre_rendering(Camera const& camera,
                     SerializedScene const& scene,
                     CameraMode eye,
                     RenderContext const& ctx);

  /**
   *
   */
  void apply_material_mapping(
      std::set<std::string> const& materials,
      std::vector<LayerMapping const*> const& inputs) const;

  /**
   *
   */
  /* virtual */ LayerMapping const* get_gbuffer_mapping() const;

 protected:

  /**
   *
   */
  /* virtual */ void rendering(Camera const& camera,
                               SerializedScene const& scene,
                               CameraMode eye,
                               RenderContext const& ctx);

 private:

  /**
   *
   */
  void set_uniforms(SerializedScene const& scene, RenderContext const& ctx);

  std::unique_ptr<FinalUberShader> shader_;
};

}

#endif  // GUA_FINAL_PASS_HPP
