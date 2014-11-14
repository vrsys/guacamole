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

#ifndef GUA_PIPELINE_PASS_HPP
#define GUA_PIPELINE_PASS_HPP

#include <scm/gl_core.h>
#include <gua/renderer/ShaderProgram.hpp>

namespace gua {

class Pipeline;
class PipelinePass;
class RenderContext;

enum class RenderMode {
  Custom, Callback, Quad
};

class PipelinePassDescription {
 public:

  virtual PipelinePassDescription* make_copy() const = 0;
  virtual ~PipelinePassDescription() {}

  friend class Pipeline;
  friend class PipelinePass;

 protected:
  // shader names
  std::string vertex_shader_ = "";
  std::string fragment_shader_ = "";
  std::string geometry_shader_ = "";

  bool needs_color_buffer_as_input_ = false;
  bool writes_only_color_buffer_ = false;
  bool doClear_ = false;

  RenderMode rendermode_ = RenderMode::Custom;

  boost::optional<scm::gl::rasterizer_state_desc> rasterizer_state_;
  boost::optional<scm::gl::blend_state_desc> blend_state_;
  boost::optional<scm::gl::depth_stencil_state_desc> depth_stencil_state_;

  std::function<void(PipelinePass&, PipelinePassDescription const& , Pipeline&)>
    process_ = [](PipelinePass&, PipelinePassDescription const&, Pipeline&) {
      return;
    };
 public:
  std::map<std::string, float> uniforms1f;
};

class PipelinePass {
 public:

  inline bool needs_color_buffer_as_input() const {
    return needs_color_buffer_as_input_;
  }
  inline bool writes_only_color_buffer() const {
    return writes_only_color_buffer_;
  }

  void process(PipelinePassDescription const& desc, Pipeline& pipe);
  virtual void on_delete(Pipeline* pipe) {}

  friend class Pipeline;

 protected:
 public:  // for refactoring purposes
  PipelinePass() {}
  PipelinePass(PipelinePassDescription const&, RenderContext const&);
  ~PipelinePass() {}

  std::shared_ptr<ShaderProgram> shader_ = nullptr;

  scm::gl::rasterizer_state_ptr rasterizer_state_ = nullptr;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_ = nullptr;
  scm::gl::blend_state_ptr blend_state_ = nullptr;

  bool needs_color_buffer_as_input_ = false;
  bool writes_only_color_buffer_ = false;
  bool doClear_ = false;
  RenderMode rendermode_ = RenderMode::Custom;

  std::function<void(PipelinePass&, PipelinePassDescription const&, Pipeline&)>
    process_ = [](PipelinePass&, PipelinePassDescription const&, Pipeline&) {
      return;
    };
};

}

#endif  // GUA_PIPELINE_PASS_HPP
