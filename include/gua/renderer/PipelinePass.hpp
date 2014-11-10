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

class PipelinePassDescription {
 public:

  virtual PipelinePassDescription* make_copy() const = 0;

  friend class Pipeline;
 protected:
  virtual PipelinePass make_pass(RenderContext const& ctx) const = 0;
};


class PipelinePass {
 public:

  inline bool needs_color_buffer_as_input() const {
    return needs_color_buffer_as_input_;
  }
  inline bool writes_only_color_buffer()    const {
    return writes_only_color_buffer_;
  }

  virtual void process(Pipeline& pipe) {
    process_(*this, pipe);
  }
  virtual void on_delete(Pipeline* pipe) {};

  friend class Pipeline;

 protected:
 public: // for refactoring purposes
  PipelinePass() {}
  ~PipelinePass() {}

  const PipelinePassDescription* description_ = nullptr;
  std::shared_ptr<ShaderProgram> shader_ = nullptr;

  scm::gl::rasterizer_state_ptr rasterizer_state_ = nullptr;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_ = nullptr;
  scm::gl::blend_state_ptr blend_state_ = nullptr;

  bool needs_color_buffer_as_input_ = false;
  bool writes_only_color_buffer_ = false;

  std::function<void(PipelinePass&, Pipeline&)> process_ =
    [](PipelinePass&,Pipeline&) { return; };
};

}

#endif  // GUA_PIPELINE_PASS_HPP
