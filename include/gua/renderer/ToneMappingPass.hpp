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

#ifndef GUA_TONEMAPPING_PASS_HPP
#define GUA_TONEMAPPING_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

namespace gua {

class Pipeline;

class GUA_DLL ToneMappingPassDescription : public PipelinePassDescription {
 public:
   enum class Method {
     LINEAR = 0,
     HEJL = 1
   };
  ToneMappingPassDescription();

  ToneMappingPassDescription& exposure(float);
  float exposure() const;
  ToneMappingPassDescription& method(Method);
  ToneMappingPassDescription::Method method() const;


  PipelinePassDescription* make_copy() const override {
    return new ToneMappingPassDescription(*this);
  }
  friend class Pipeline;
 protected:
  PipelinePass make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) override {
    return PipelinePass{*this, ctx, substitution_map};
  }
};

}

#endif  // GUA_TONEMAPPING_PASS_HPP
