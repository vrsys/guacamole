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
#include <gua/renderer/RenderTarget.hpp>

#include <scm/gl_core/render_device/opengl/gl_core.h>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

RenderTarget::RenderTarget(math::vec2ui const& resolution) : resolution_(resolution) {}

////////////////////////////////////////////////////////////////////////////////

void RenderTarget::set_viewport(RenderContext const& ctx)
{
    if(ctx.render_context)
    {
        ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2f(0, 0), scm::math::vec2f(resolution_)));
    }
}


////////////////////////////////////////////////////////////////////////////////

void RenderTarget::set_side_by_side_viewport_array_internal(RenderContext const& ctx)
{
    if(ctx.render_context)
    {
        auto viewport_array = scm::gl::viewport(scm::math::vec2f(0, 0), scm::math::vec2f(resolution_.x/2, resolution_.y))(scm::math::vec2f(resolution_.x/2, 0), scm::math::vec2f(resolution_.x/2, resolution_.y));
        ctx.render_context->set_viewports(viewport_array);
    }
}

////////////////////////////////////////////////////////////////////////////////

void RenderTarget::set_side_by_side_viewport_array(RenderContext const& ctx)
{
    if(ctx.render_context)
    {
    	if (ctx.mode == gua::CameraMode::BOTH)
    	{
            set_side_by_side_viewport_array_internal(ctx);
    	} else {
    		set_viewport(ctx);
    	}

        //glapi.glViewportIndexedf(0, resolution_.x/2, 0, resolution_.x + resolution_.x/2, resolution_.y);
        //glapi.glViewportIndexedf(1, resolution_.x, 0, resolution_.x, resolution_.y);
        //ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2f(0, 0), scm::math::vec2f(resolution_)));
    }
}

////////////////////////////////////////////////////////////////////////////////

void RenderTarget::unbind(RenderContext const& ctx) { ctx.render_context->reset_framebuffer(); }

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
