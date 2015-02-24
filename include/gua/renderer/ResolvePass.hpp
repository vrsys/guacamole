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

#ifndef GUA_RESOLVE_PASS_HPP
#define GUA_RESOLVE_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua {

class Pipeline;

class GUA_DLL ResolvePassDescription : public PipelinePassDescription {
 public:

  enum class BackgroundMode {
    COLOR = 0,
    SKYMAP_TEXTURE = 1,
    QUAD_TEXTURE = 2,
  };

  enum class ToneMappingMethod {
    LINEAR = 0,
    HEJL = 1,
    REINHARD = 2
  };

  enum class EnvironmentLightingMode {
    SPHEREMAP = 0,
    CUBEMAP = 1,
    AMBIENT_COLOR = 2
  };

  ResolvePassDescription();

  ResolvePassDescription& color(utils::Color3f const& color);
  utils::Color3f color() const;

  ResolvePassDescription& texture(std::string const& texture);
  std::string texture() const;

  ResolvePassDescription& environment_lighting (std::string const& spheremap_texture);

  ResolvePassDescription& environment_lighting (std::string const& cube_map_positive_x, 
                                                std::string const& cube_map_negative_x, 
                                                std::string const& cube_map_positive_y,
                                                std::string const& cube_map_negative_y,
                                                std::string const& cube_map_positive_z,
                                                std::string const& cube_map_negative_z);

  ResolvePassDescription& environment_lighting (utils::Color3f const& color);

  ResolvePassDescription& environment_lighting_method (EnvironmentLightingMode mode);
  EnvironmentLightingMode environment_lighting_method() const;

  ResolvePassDescription& mode(BackgroundMode mode);
  BackgroundMode mode() const;

  ResolvePassDescription& enable_fog(bool enable_fog);
  bool enable_fog() const;

  ResolvePassDescription& fog_start(float fog_start);
  float fog_start() const;

  ResolvePassDescription& fog_end(float fog_end);
  float fog_end() const;

  ResolvePassDescription& tone_mapping_exposure(float value);
  float tone_mapping_exposure() const;

  ResolvePassDescription& tone_mapping_method(ToneMappingMethod value);
  ToneMappingMethod tone_mapping_method() const;

  ResolvePassDescription& debug_tiles(bool value);
  bool debug_tiles() const;

  std::shared_ptr<PipelinePassDescription> make_copy() const override;
  friend class Pipeline;

 protected:
  PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

  ToneMappingMethod tone_mapping_method_             = ToneMappingMethod::LINEAR;
  BackgroundMode background_mode_                    = BackgroundMode::COLOR;
  EnvironmentLightingMode environment_lighting_mode_ = EnvironmentLightingMode::AMBIENT_COLOR;
  bool debug_tiles_                                  = false;
};  

}

#endif  // GUA_RESOLVE_PASS_HPP
