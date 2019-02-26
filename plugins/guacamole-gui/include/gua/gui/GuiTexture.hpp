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

#ifndef GUA_GUI_TEXTURE_HPP
#define GUA_GUI_TEXTURE_HPP

// guacamole headers
#include <gua/renderer/Texture2D.hpp>

namespace Awesomium
{
class WebView;
}

namespace gua
{
/**
 * A class representing a texture.
 *
 * This class allows to load texture data from a file and bind the
 * texture to an OpenGL context.
 */
class GUA_DLL GuiTexture : public Texture2D
{
  public:
    /**
     * Constructor.
     */
    GuiTexture(unsigned width, unsigned height, Awesomium::WebView* view);

    math::vec2ui const get_handle(RenderContext const& context) const override;

  protected:
    Awesomium::WebView* view_ = nullptr;
};

} // namespace gua
#endif // GUA_GUI_TEXTURE_HPP
