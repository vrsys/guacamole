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

  enum BackgroundMode {
    COLOR = 0,
    SKYMAP_TEXTURE = 1,
    QUAD_TEXTURE = 2,
  };

  enum class ToneMappingMethod {
    LINEAR = 0,
    HEJL = 1
  };

  ResolvePassDescription();

  ResolvePassDescription& color(utils::Color3f const& color);
  utils::Color3f color() const;

  ResolvePassDescription& texture(std::string const& texture);
  std::string texture() const;

  ResolvePassDescription& mode(BackgroundMode const& mode);
  BackgroundMode mode() const;


  ResolvePassDescription& enable_fog(bool enable_fog);
  bool enable_fog() const;

  ResolvePassDescription& fog_start(float fog_start);
  float fog_start() const;

  ResolvePassDescription& fog_end(float fog_end);
  float fog_end() const;

  ResolvePassDescription& tone_mapping_exposure(float value) {
    tone_mapping_exposure_ = value; return *this; }
  float tone_mapping_exposure() const { return tone_mapping_exposure_; }

  ResolvePassDescription& tone_mapping_method(ToneMappingMethod value) {
    tone_mapping_method_ = value; return *this; }
  ToneMappingMethod tone_mapping_method() const { return tone_mapping_method_; }

  ResolvePassDescription& debug_tiles(bool value) {
    debug_tiles_ = value; return *this; }
  bool debug_tiles() const { return debug_tiles_; }

  PipelinePassDescription* make_copy() const override;
  friend class Pipeline;

 protected:
  PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

  ToneMappingMethod tone_mapping_method_ = ToneMappingMethod::LINEAR;
  float tone_mapping_exposure_           = 1.f;
  bool debug_tiles_                      = false;
};

}

#endif  // GUA_RESOLVE_PASS_HPP
