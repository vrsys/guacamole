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

#ifndef GUA_CAPTURE_HPP
#define GUA_CAPTURE_HPP

// guacamole headers
#include <gua/renderer/WindowBase.hpp>
#include <boost/thread.hpp>

namespace gua {

/**
 * A window for displaying stuff.
 *
 * It's a window which can display OpenGL stuff.
 */
class GUA_DLL CaptureWindow : public WindowBase {
 public:

  enum class RepMsgType : int32_t { error = 0, image = 1 };

  struct RepImage {
    RepMsgType type = RepMsgType::image;
    int32_t width;
    int32_t height;
    uint32_t bpp;
    uint32_t gl_internal_format;
    uint32_t gl_base_format;
    uint32_t gl_type;
    std::vector<char> data;

    size_t size_header() const {
      return sizeof(type) + sizeof(width) + sizeof(height) + sizeof(bpp) +
             sizeof(gl_internal_format) + sizeof(gl_base_format) +
             sizeof(gl_type);
    }
    size_t size() const { return size_header() + data.size(); }
  };


  /**
   * Constructor.
   *
   * Creates a new CaptureWindow. It owns a RenderContext where Geomtries
   * can be drawn to.
   *
   * \param description   The description of the window.
   */
  CaptureWindow(Configuration const& configuration = Configuration());

  /**
   * Destructor.
   *
   * Cleans all associated memory.
   */
  virtual ~CaptureWindow();

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

  void set_image(RepImage img);
  RepImage get_image();

 private:
  scm::gl::wm::window_ptr window_;
  RepImage image_;
  boost::shared_mutex mutex_;
};

}

#endif  // GUA_CAPTURE_HPP
