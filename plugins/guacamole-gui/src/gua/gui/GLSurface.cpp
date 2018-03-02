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

#include <gua/gui/GLSurface.hpp>

#include <gua/gui/GuiTexture.hpp>

namespace gua {

 ///////////////////////////////////////////////////////////////////////////////
 // ----------------------------------------------------------- public interface

  // ----------------------------------------------------- contruction interface
  GLSurface::GLSurface(unsigned width, unsigned height)
    : buffer_(width * height * 4)
    , width_(width)
    , height_(height)
    , needs_update_() {
      std::cout << "Its-a-me-Surface!" << std::endl;
    }


  // ------------------------------------------------------------ public methods

  //////////////////////////////////////////////////////////////////////////////

  bool GLSurface::bind(RenderContext const& ctx, const GuiTexture* gui_texture) {
    while (needs_update_.size() <= ctx.id) {
      needs_update_.push_back(true);
    }


    gui_texture->update_sub_data(
      ctx,
      scm::gl::texture_region(math::vec3ui(0, 0, 0), math::vec3ui(width_, height_, 1)),
      0u, scm::gl::FORMAT_BGRA_8, &buffer_.front());

    
    if (needs_update_[ctx.id]) {
      std::unique_lock<std::mutex> lock(mutex_);
      needs_update_[ctx.id] = false;
      gui_texture->update_sub_data(
        ctx,
        scm::gl::texture_region(math::vec3ui(0, 0, 0), math::vec3ui(width_, height_, 1)),
        0u, scm::gl::FORMAT_BGRA_8, &buffer_.front()
      );
      
    }

    

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  
  void GLSurface::OnPaint(CefRefPtr<CefBrowser> browser, PaintElementType type, const RectList &dirtyRects, const void *buffer, int width, int height)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    for (int r = 0; r < height_; r++) {
      auto row(height_ - r  - 1);
      memcpy(&buffer_.front() + row * width_*4, buffer + r * width_ * 4, width_ * 4);
    }

    for (uint32_t i(0); i < needs_update_.size(); ++i) {
      needs_update_[i] = true;
    }
  }
  

  bool GLSurface::GetViewRect(CefRefPtr<CefBrowser> browser, CefRect &rect)
  {
      rect = CefRect(0, 0, width_, height_);
      return true;
  }


} //namespace gua
