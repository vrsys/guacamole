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

#ifndef GUA_SSAO_PASS_HPP
#define GUA_SSAO_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/renderer/BuiltInTextures.hpp>

#include <memory>

namespace gua {

class Pipeline;

class SSAOPassDescription : public PipelinePassDescription {
 public:

  SSAOPassDescription();
  SSAOPassDescription(SSAOPassDescription const& copy) = default;

  float                radius() const;
  SSAOPassDescription& radius(float radius);

  float                intensity() const;
  SSAOPassDescription& intensity(float intensity);

  float                falloff() const;
  SSAOPassDescription& falloff(float falloff);

  virtual PipelinePassDescription* make_copy() const;

  friend class Pipeline;

 protected:
  float radius_;
  float intensity_;
  float falloff_;
};

}

#endif  // GUA_SSAO_PASS_HPP
