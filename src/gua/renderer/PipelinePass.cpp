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

// class header
#include <gua/renderer/PipelinePass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

void PipelinePass::process(PipelinePassDescription* desc, Pipeline& pipe) {
  if (RenderMode::Custom == rendermode_) {
    process_(*this, desc, pipe);
  } else if (RenderMode::Callback == rendermode_) {
    auto const& ctx(pipe.get_context());
    pipe.get_gbuffer().bind(ctx, this);
    pipe.get_gbuffer().set_viewport(ctx);
    if (doClear_)
      pipe.get_gbuffer().clear_color(ctx);
    if (depth_stencil_state_)
      ctx.render_context->set_depth_stencil_state(depth_stencil_state_);
    if (blend_state_)
      ctx.render_context->set_blend_state(blend_state_);
    if (rasterizer_state_)
      ctx.render_context->set_rasterizer_state(rasterizer_state_);
    shader_->use(ctx);
    process_(*this, desc, pipe);
    pipe.get_gbuffer().unbind(ctx);
    ctx.render_context->reset_state_objects();
  }
}

}
