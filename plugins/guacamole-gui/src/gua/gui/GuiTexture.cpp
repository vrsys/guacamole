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

#include <gua/gui/GLSurface.inl>

#include <Awesomium/WebCore.h>

#include <include/cef_client.h>

namespace gua {

GuiTexture::GuiTexture(unsigned width, unsigned height, CefRefPtr<GuiBrowserClient> browserClient)
    : Texture2D(width, height, scm::gl::FORMAT_RGBA_8)
    , browserClient_(browserClient)
  {}

math::vec2ui const GuiTexture::get_handle(RenderContext const& context) const {
  
  if (browserClient_ == nullptr) {
    return math::vec2ui(0, 0);
  }

  auto surface = static_cast<GLSurface*>(browserClient_->GetRenderHandler().get());

  if (surface == nullptr) {
    return math::vec2ui(0, 0);
  }

  surface->bind(context, this);
  
  return Texture2D::get_handle(context);
  
}

}
