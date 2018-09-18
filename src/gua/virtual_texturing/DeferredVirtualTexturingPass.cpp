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
#include <gua/virtual_texturing/DeferredVirtualTexturingPass.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {
namespace vt {

////////////////////////////////////////////////////////////////////////////////
DeferredVirtualTexturingPassDescription::DeferredVirtualTexturingPassDescription()
  : PipelinePassDescription()
{
  vertex_shader_ = "shaders/common/fullscreen_quad.vert";
  fragment_shader_ = "shaders/resolve.frag";
  name_ = "DeferredVirtualTexturingPass";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = true;
  rendermode_ = RenderMode::Quad;
  depth_stencil_state_ = boost::make_optional(
    scm::gl::depth_stencil_state_desc(
      false, false, scm::gl::COMPARISON_LESS, true, 1, 0,
      scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    )
  );
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PipelinePassDescription> DeferredVirtualTexturingPassDescription::make_copy() const {
  return std::make_shared<DeferredVirtualTexturingPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass DeferredVirtualTexturingPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
  PipelinePass pass{*this, ctx, substitution_map};
  return pass;
}

}
}
