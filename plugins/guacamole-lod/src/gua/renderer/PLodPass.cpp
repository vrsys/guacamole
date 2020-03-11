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
#include <gua/renderer/PLodPass.hpp>

// guacamole headers
#include <gua/renderer/LodResource.hpp>
#include <gua/renderer/PLodRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>

#include <scm/gl_core/shader_objects.h>

// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>

namespace gua
{

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PLodPassDescription> const PipelineDescription::get_plod_pass() const { return get_pass_by_type<PLodPassDescription>(); }


////////////////////////////////////////////////////////////////////////////////

PLodPassDescription::PLodPassDescription(SurfelRenderMode const mode) : PipelinePassDescription(), surfel_render_mode_(mode)
{
    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = false;
    private_.enable_for_shadows_ = true;
    private_.rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////
PLodPassDescription& PLodPassDescription::mode(SurfelRenderMode const mode)
{
    surfel_render_mode_ = mode;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
PLodPassDescription::SurfelRenderMode PLodPassDescription::mode() const { return surfel_render_mode_; }

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> PLodPassDescription::make_copy() const { return std::make_shared<PLodPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

PipelinePass PLodPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    auto renderer = std::make_shared<PLodRenderer>();
    renderer->set_global_substitution_map(substitution_map);

    private_.process_ = [renderer](PipelinePass& pass, PipelinePassDescription const& desc, Pipeline& pipe) {
        renderer->render_switch_occlusion_culling(pipe, desc);
    };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
