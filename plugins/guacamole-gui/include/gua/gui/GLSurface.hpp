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

#ifndef GUA_GUI_GL_SURFACE_
#define GUA_GUI_GL_SURFACE_

#include <gua/renderer/RenderContext.hpp>

#include <Awesomium/BitmapSurface.h>

#include <include/cef_app.h>
#include <include/cef_client.h>
#include <include/cef_render_handler.h>

#include <mutex>

namespace gua {

class GuiTexture;

class GLSurface : public CefRenderHandler {

  ///////////////////////////////////////////////////////////////////////////////
  // ----------------------------------------------------------- public interface
  public:

    // ----------------------------------------------------- contruction interface
    GLSurface(unsigned width, unsigned height);


    // ------------------------------------------------------------ public methods

    //////////////////////////////////////////////////////////////////////////////

    bool bind(RenderContext const& ctx, const GuiTexture* gui_texture);

    //////////////////////////////////////////////////////////////////////////////
    /*
    void Paint(unsigned char* src_buffer, int src_row_span,
             Awesomium::Rect const& src_rect,
             Awesomium::Rect const& dest_rect);
    */



    //////////////////////////////////////////////////////////////////////////////

    //void Scroll(int dx, int dy);

    ///////////////////////////////////////////////////////////////////////////////

    bool GetViewRect(CefRefPtr<CefBrowser> browser, CefRect &rect);


    ///////////////////////////////////////////////////////////////////////////////

    void OnPaint(CefRefPtr<CefBrowser> browser, PaintElementType type, const RectList &dirtyRects, const void *buffer, int width, int height);

    ///////////////////////////////////////////////////////////////////////////////


  public:
    IMPLEMENT_REFCOUNTING(GLSurface);

  // ---------------------------------------------------------- private interface
  private:

    std::vector<unsigned char>  buffer_;

    unsigned   width_;
    unsigned   height_;
    std::mutex mutex_;
    std::vector<bool> needs_update_;
};

}
#endif  // GUA_GUI_GL_SURFACE_
