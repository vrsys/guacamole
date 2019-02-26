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
#include <gua/gui/GuiTexture.hpp>

#include "GLSurface.inl"

#include <Awesomium/WebCore.h>

namespace gua
{
GuiTexture::GuiTexture(unsigned width, unsigned height, Awesomium::WebView* view) : Texture2D(width, height, scm::gl::FORMAT_RGBA_8), view_(view) {}

math::vec2ui const GuiTexture::get_handle(RenderContext const& context) const
{
    if(view_ == nullptr)
    {
        return math::vec2ui(0, 0);
    }

    auto surface = static_cast<GLSurface*>(view_->surface());

    if(surface == nullptr)
    {
        return math::vec2ui(0, 0);
    }

    surface->bind(context, this);

    return Texture2D::get_handle(context);
}

} // namespace gua
