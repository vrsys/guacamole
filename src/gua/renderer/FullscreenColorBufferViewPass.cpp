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
#include <gua/renderer/FullscreenColorBufferViewPass.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua
{
FullscreenColorBufferViewPassDescription::FullscreenColorBufferViewPassDescription() : PipelinePassDescription()
{
    vertex_shader_ = "shaders/common/fullscreen_quad.vert";
    fragment_shader_ = "shaders/colorbufferview.frag";
    private_.name_ = "FullscreenColorBufferViewPass";

    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = true;
    private_.rendermode_ = RenderMode::Quad;
    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(false, false));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> FullscreenColorBufferViewPassDescription::make_copy() const { return std::make_shared<FullscreenColorBufferViewPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass FullscreenColorBufferViewPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
