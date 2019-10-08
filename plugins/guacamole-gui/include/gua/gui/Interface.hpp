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

#ifndef GUA_GUI_INTERFACE_HPP
#define GUA_GUI_INTERFACE_HPP

// includes  -------------------------------------------------------------------
#include <gua/utils/Singleton.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/events/Signal.hpp>
#include <gua/gui/mouse_enums.hpp>

// forward declares ------------------------------------------------------------
namespace Awesomium
{
class WebCore;
class WebView;
class WebSession;
} // namespace Awesomium

namespace gua
{
class ShaderProgram;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// -----------------------------------------------------------------------------
class GUA_DLL Interface : public Singleton<Interface>
{
    ///////////////////////////////////////////////////////////////////////////////
    // ----------------------------------------------------------- public interface
  public:
    events::Signal<Cursor> on_cursor_change;

    void update() const;

    friend class GuiResource;
    friend class Singleton<Interface>;

    ///////////////////////////////////////////////////////////////////////////////
    // ---------------------------------------------------------- private interface
  private:
    // this class is a Singleton --- private c'tor and d'tor
    Interface();
    ~Interface();

    Awesomium::WebView* create_webview(int width, int height) const;

    Awesomium::WebCore* web_core_;
    Awesomium::WebSession* web_session_;
};

// -----------------------------------------------------------------------------

} // namespace gua

#endif // GUA_GUI_INTERFACE_HPP
