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
#include "gua/renderer/RenderContext.hpp"

#include <gua/renderer/WindowBase.hpp>

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////

  RenderContext::RenderContext()
    : context(),
    display(),
    render_context(),
    render_device(),
    render_window(nullptr),
    id(0),
    framecount(0),
    bone_transform_blocks{}
    {}
  RenderContext::RenderContext(RenderContext const& ctx)
    : context(ctx.context),
    display(ctx.display),
    render_context(ctx.render_context),
    render_device(ctx.render_device),
    render_window(ctx.render_window),
    id(ctx.id),
    framecount(ctx.framecount),
    bone_transform_blocks{ctx.bone_transform_blocks}
    {}
}
