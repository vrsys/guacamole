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

enum class RenderMode { Callback };

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

  void process(PipelinePassDescription* desc, Pipeline& pipe) {
#if 0
    auto const& ctx(pipe.get_context());
    pipe.get_gbuffer().bind(ctx, &pass);
    pipe.get_gbuffer().set_viewport(ctx);
    //pipe.get_gbuffer().clear_color(ctx);
    if (depth_stencil_state_)
      ctx.render_context->set_depth_stencil_state(pass.depth_stencil_state_);
    if (blend_state_)
      ctx.render_context->set_blend_state(pass.blend_state_);
    if (rasterizer_state_)
      ctx.render_context->set_rasterizer_state(rasterizer_state_);
    shader_->use(ctx);
#endif
    process_(*this, desc, pipe);
#if 0
    pipe.get_gbuffer().unbind(ctx);
    ctx.render_context->reset_state_objects();
#endif
  }
  virtual void on_delete(Pipeline* pipe) {};

  friend class Pipeline;

 protected:
 public: // for refactoring purposes
  PipelinePass() {}
  ~PipelinePass() {}

  std::shared_ptr<ShaderProgram> shader_ = nullptr;

  scm::gl::rasterizer_state_ptr rasterizer_state_ = nullptr;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_ = nullptr;
  scm::gl::blend_state_ptr blend_state_ = nullptr;

  bool needs_color_buffer_as_input_ = false;
  bool writes_only_color_buffer_ = false;

  std::function<void(PipelinePass&, PipelinePassDescription* desc, Pipeline&)> process_ =
    [](PipelinePass&, PipelinePassDescription*, Pipeline&) { return; };
  RenderMode rendermode_ = RenderMode::Callback;
};

}

#endif  // GUA_PIPELINE_PASS_HPP
