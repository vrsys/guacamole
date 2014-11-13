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
#include <gua/renderer/EmissivePass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShadowMapBuffer.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

EmissivePassDescription::EmissivePassDescription()
  : PipelinePassDescription() {
  vertex_shader_ = "shaders/lighting_emit.vert";
  fragment_shader_ = "shaders/lighting_emit.frag";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = true;
  doClear_ = true;
  rendermode_ = RenderMode::Callback; // RenderMode::Quad;

  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));
  blend_state_ = boost::make_optional(
      scm::gl::blend_state_desc(scm::gl::blend_ops(true,
                                                    scm::gl::FUNC_ONE,
                                                    scm::gl::FUNC_ONE,
                                                    scm::gl::FUNC_ONE,
                                                    scm::gl::FUNC_ONE)));
  process_ = [](
      PipelinePass & pass, PipelinePassDescription * desc, Pipeline & pipe) {
    auto const& ctx(pipe.get_context());
    pass.shader_->set_uniform(ctx, 1.0f / pipe.get_gbuffer().get_width(), "gua_texel_width");
    pass.shader_->set_uniform(ctx, 1.0f / pipe.get_gbuffer().get_height(), "gua_texel_height");
    pass.shader_->set_uniform(ctx, pipe.get_gbuffer().get_current_color_buffer()->get_handle(ctx), "gua_gbuffer_color");
    pass.shader_->set_uniform(ctx, pipe.get_gbuffer().get_current_pbr_buffer()->get_handle(ctx), "gua_gbuffer_pbr");
    //pipe.bind_gbuffer_input(pass.shader_);
    pipe.draw_quad();
  };
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* EmissivePassDescription::make_copy() const {
  return new EmissivePassDescription(*this);
}

}
