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

#ifndef GUA_WINDOW_HPP
#define GUA_WINDOW_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/WarpMatrix.hpp>
#include <gua/renderer/enums.hpp>
#include <gua/math/math.hpp>
#include <gua/utils/configuration_macro.hpp>

// external headers
#include <atomic>
#include <memory>
#include <string>
#include <scm/gl_util/primitives/quad.h>

namespace gua {

class Geometry;
class Texture2D;
class StereoBuffer;

/**
 * A window for displaying stuff.
 *
 * It's a window which can display OpenGL stuff.
 */
class GUA_DLL Window {
 public:

  enum TextureDisplayMode {
    FULL,
    RED,
    GREEN,
    CYAN,
    CHECKER_EVEN,
    CHECKER_ODD
  };

  /**
   * A description of a window.
   *
   * Used when creating a window.
   */
  struct Configuration {

    GUA_ADD_PROPERTY(math::vec2ui, size, math::vec2ui(800, 600));
    GUA_ADD_PROPERTY(std::string, title, "guacamole");
#if WIN32
    GUA_ADD_PROPERTY(std::string, display_name, "\\\\.\\DISPLAY1");
#else
    GUA_ADD_PROPERTY(std::string, display_name, ":0.0");
#endif
    GUA_ADD_PROPERTY(int, monitor, 0);
    GUA_ADD_PROPERTY(StereoMode, stereo_mode, StereoMode::MONO);
    GUA_ADD_PROPERTY(math::vec2ui, left_resolution, math::vec2ui(800, 600));
    GUA_ADD_PROPERTY(math::vec2ui, left_position, math::vec2ui(0, 0));
    GUA_ADD_PROPERTY(math::vec2ui, right_resolution, math::vec2ui(800, 600));
    GUA_ADD_PROPERTY(math::vec2ui, right_position, math::vec2ui(0, 0));
    GUA_ADD_PROPERTY(bool, enable_vsync, true);
    GUA_ADD_PROPERTY(bool, debug, false);
    GUA_ADD_PROPERTY(std::string, warp_matrix_red_right, "");
    GUA_ADD_PROPERTY(std::string, warp_matrix_green_right, "");
    GUA_ADD_PROPERTY(std::string, warp_matrix_blue_right, "");
    GUA_ADD_PROPERTY(std::string, warp_matrix_red_left, "");
    GUA_ADD_PROPERTY(std::string, warp_matrix_green_left, "");
    GUA_ADD_PROPERTY(std::string, warp_matrix_blue_left, "");
  } config;

  /**
   * Constructor.
   *
   * Creates a new Window. It owns a RenderContext where Geomtries
   * can be drawn to.
   *
   * \param description   The description of the window.
   */
  Window(Configuration const& configuration = Configuration());

  /**
   * Destructor.
   *
   * Cleans all associated memory.
   */
  virtual ~Window();

  virtual void open();
  virtual bool get_is_open() const = 0;

  virtual void create_shader();

  virtual void close() = 0;

  /**
   * Activate the context of this window.
   *
   * Makes the RenderContext of this window current. All preceeding
   * OpenGL calls will be invoked on this window.
   */
  virtual void set_active(bool active) const = 0;

  /**
   * Starts the drawing of a new frame.
   *
   * This should be called when a new frame is about to be drawn.
   */
  virtual void start_frame() const;

  /**
   * Ends the drawing of a new frame.
   *
   * This should be called when drawing a frame has been done.
   */
  virtual void finish_frame() const = 0;

  /**
   *
   */
  virtual void display(std::shared_ptr<Texture2D> const& center_texture);

  virtual void display(std::shared_ptr<Texture2D> const& left_texture,
                       std::shared_ptr<Texture2D> const& right_texture);

  /**
   * Get the RenderContext of this window.
   *
   * Can be called in order to retrieve the RenderContext of this
   * Window.
   *
   * \return The context owned by this window.
   */
  RenderContext* get_context();

protected:

  struct DebugOutput : public scm::gl::render_context::debug_output {
    /*virtual*/ void operator()(scm::gl::debug_source source,
                                scm::gl::debug_type type,
                                scm::gl::debug_severity severity,
                                const std::string& message) const;
  };

  ShaderProgram fullscreen_shader_;
  scm::gl::quad_geometry_ptr fullscreen_quad_;

  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
  scm::gl::blend_state_ptr blend_state_;
  RenderContext ctx_;

 private:
  void display(std::shared_ptr<Texture2D> const& texture,
               math::vec2ui const& size,
               math::vec2ui const& position,
               TextureDisplayMode mode = FULL,
               bool is_left = true,
               bool clear = true);


  static std::atomic_uint last_context_id_;

  static std::mutex last_context_id_mutex_;

  std::shared_ptr<WarpMatrix> warpRR_, warpGR_, warpBR_, warpRL_, warpGL_, warpBL_;
};

}

#endif  // GUA_WINDOW_HPP
