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
#include <gua/renderer/VTResponsibility.hpp>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING

namespace gua
{
VTPreResponsibilityDescription::VTPreResponsibilityDescription(VTRenderer& vt_renderer)
{
    private_.name_ = "vt_pre_responsibility";
    private_.type_ = PipelineResponsibilityPrivate::TYPE::PRE_RENDER;
    private_.fulfil_ = [vt_renderer](Pipeline& pipe) { vt_renderer.pre_render(pipe); };
}
VTPostResponsibilityDescription::VTPostResponsibilityDescription(VTRenderer& vt_renderer)
{
    private_.name_ = "vt_post_responsibility";
    private_.type_ = PipelineResponsibilityPrivate::TYPE::POST_RENDER;
    private_.fulfil_ = [vt_renderer](Pipeline& pipe) { vt_renderer.post_render(pipe); };
}
} // namespace gua

#endif
