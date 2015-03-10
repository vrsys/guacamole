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

#ifndef GUA_HEADLESS_SURFACE_HPP
#define GUA_HEADLESS_SURFACE_HPP

// guacamole headers
#include <gua/renderer/WindowBase.hpp>

namespace gua {

/**
 * A window for displaying stuff.
 *
 * It's a window which can display OpenGL stuff.
 */
class GUA_DLL HeadlessSurface : public WindowBase {
 public:

  /**
   * Constructor.
   *
   * Creates a new HeadlessSurface. It owns a RenderContext where Geomtries
   * can be drawn to.
   *
   * \param description   The description of the window.
   */
  HeadlessSurface(Configuration const& configuration = Configuration());

  /**
   * Destructor.
   *
   * Cleans all associated memory.
   */
  virtual ~HeadlessSurface();

  virtual void open();
  virtual bool get_is_open() const;
  virtual bool should_close() const;
  virtual void close();

  virtual void process_events() { }

  /**
   * Activate the context of this window.
   *
   * Makes the RenderContext of this window current. All preceeding
   * OpenGL calls will be invoked on this window.
   */
  virtual void set_active(bool active);

  /**
   * Ends the drawing of a new frame.
   *
   * This should be called when drawing a frame has been done.
   */
  virtual void finish_frame() const;

  struct Image {
    int32_t width = 0;
    int32_t height = 0;
    uint32_t bpp = 0;
    uint32_t gl_internal_format = 0;
    uint32_t gl_base_format = 0;
    uint32_t gl_type = 0;
    std::vector<char> data = {};

    size_t size_header() const {
    return sizeof(width) + sizeof(height) + sizeof(bpp) +
           sizeof(gl_internal_format) + sizeof(gl_base_format) +
           sizeof(gl_type);
    }
    size_t size() const { return size_header() + data.size(); }
  };

  void set_image(Image);
  Image get_image();

 private:
  scm::gl::wm::window_ptr window_ = nullptr;
  scm::gl::wm::headless_surface_ptr headless_surface_ = nullptr;
  Image image_;
  boost::shared_mutex mutex_;
};

}

#endif  // GUA_WINDOW_HPP
