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
#include <gua/spoints/SPointsPass.hpp>

#include <gua/spoints/SPointsFeedbackCollector.hpp>
#include <gua/spoints/SPointsNode.hpp>
#include <gua/spoints/SPointsRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/Resources.hpp>

namespace gua
{
FeedbackCollectorResponsibilityDescription::FeedbackCollectorResponsibilityDescription()
{
    private_.name_ = "spoints_feedback_collector_responsibility";
    private_.type_ = PipelineResponsibilityPrivate::TYPE::POST_RENDER;
    private_.fulfil_ = [](Pipeline& pipe) {
        SPointsFeedbackCollector::instance()->send_feedback_frame(pipe.get_context());
    };

}
SPointsPassDescription::SPointsPassDescription() : PipelinePassDescription()
{
    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = false;
    private_.enable_for_shadows_ = true;
    private_.rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass SPointsPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    pipeline_responsibilities_.push_back(std::make_shared<FeedbackCollectorResponsibilityDescription>());

    auto renderer{std::make_shared<SPointsRenderer>()};
    renderer->set_global_substitution_map(substitution_map);
    private_.process_ = [renderer](PipelinePass& pass, PipelinePassDescription const& desc, Pipeline& pipe, bool render_multiview) { renderer->render(pipe, desc); };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> SPointsPassDescription::make_copy() const { return std::make_shared<SPointsPassDescription>(*this); }

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
