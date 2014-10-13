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

class SSAOPass : public PipelinePass {
 public:

  virtual bool needs_color_buffer_as_input() const { return false; }
  virtual bool writes_only_color_buffer()    const { return true;  }

  virtual void process(Pipeline* pipe);

  friend class Pipeline;

  float     radius() const;
  SSAOPass& radius(float radius);

  float     intensity() const;
  SSAOPass& intensity(float intensity);

  float     falloff() const;
  SSAOPass& falloff(float falloff);

 protected:
  SSAOPass();
  ~SSAOPass() {}

 private:
  std::shared_ptr<ShaderProgram>   shader_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
  scm::gl::blend_state_ptr         blend_state_;

  float radius_;
  float intensity_;
  float falloff_;

  NoiseTexture noise_texture_;
};

}

#endif  // GUA_SSAO_PASS_HPP
