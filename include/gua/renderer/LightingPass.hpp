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

#include <gua/renderer/PipelinePass.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/ShadowMap.hpp>

#include <memory>

namespace gua {

class Pipeline;
class LightingPass;

class LightingPassDescription : public PipelinePassDescription {
 public:
  virtual PipelinePassDescription* make_copy() const;
  friend class Pipeline;

 protected:
  virtual PipelinePass* make_pass() const;
};

class LightingPass : public PipelinePass {
 public:

  virtual bool needs_color_buffer_as_input() const { return true; }
  virtual bool writes_only_color_buffer()    const { return true; }

  virtual void process(PipelinePassDescription* desc, Pipeline* pipe);

  friend class LightingPassDescription;

 protected:
  LightingPass();
  ~LightingPass() {}

 private:
  std::shared_ptr<ShaderProgram>      shader_;
  std::shared_ptr<ShaderProgram>      emit_shader_;
  std::shared_ptr<TriMeshRessource>   light_sphere_;
  std::shared_ptr<TriMeshRessource>   light_cone_;
  scm::gl::rasterizer_state_ptr       rasterizer_state_;
  scm::gl::depth_stencil_state_ptr    depth_stencil_state_;
  scm::gl::blend_state_ptr            blend_state_;

  ShadowMap                           shadow_map_;
};

}

#endif  // GUA_LIGHTING_PASS_HPP
