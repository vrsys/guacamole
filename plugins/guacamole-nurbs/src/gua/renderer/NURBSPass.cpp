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
#include <gua/renderer/NURBSPass.hpp>

// guacamole headers
#include <gua/renderer/NURBSResource.hpp>
#include <gua/renderer/NURBSRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>
#include <gpucast/core/config.hpp>

#include <scm/gl_core/shader_objects.h>

// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

NURBSPassDescription::NURBSPassDescription() : PipelinePassDescription()
{
    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = false;
    private_.enable_for_shadows_ = true;
    private_.rendermode_ = RenderMode::Custom;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(true, true, scm::gl::COMPARISON_LESS, true, 1, 0, scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)));
}

////////////////////////////////////////////////////////////////////////////////

void NURBSPassDescription::enable_pretessellation(bool enable) { _enable_pretessellation = enable; }

////////////////////////////////////////////////////////////////////////////////

bool NURBSPassDescription::enable_pretessellation() const { return _enable_pretessellation; }

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> NURBSPassDescription::make_copy() const { return std::make_shared<NURBSPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass NURBSPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    auto renderer = std::make_shared<NURBSRenderer>();
    renderer->pretessellation(_enable_pretessellation);
    renderer->set_substitutions(substitution_map);

    private_.process_ = [renderer](PipelinePass& pass, PipelinePassDescription const& desc, Pipeline& pipe) {
        pipe.get_context().render_context->set_depth_stencil_state(pass.depth_stencil_state(), 1);
        renderer->render(pipe, desc);
    };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
