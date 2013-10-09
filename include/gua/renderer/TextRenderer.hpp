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

#ifndef GUA_TEXT_RENDERER_HPP
#define GUA_TEXT_RENDERER_HPP

// guacamole headers
#include <gua/renderer/FrameBufferObject.hpp>

// external headers
#include <scm/gl_util/font/text.h>
#include <scm/gl_util/font/text_renderer.h>

namespace gua {

struct RenderContext;
class StereoBuffer;

/**
 * A class used for writing text on FBOs.
 *
 * It can write some predefined strings and information on a given FBO.
 */
class TextRenderer {
 public:

  /**
   * Constructor.
   *
   * Creates a new TextRenderer.
   *
   * \param ctx              The context to which this renderer
   *                             belongs.
   */
  TextRenderer(RenderContext const& ctx,
               float size,
               std::string const& font_name);

  /**
   * Draws the FPS on the FBO.
   *
   * Draws the frames per second on the top left corner of the FBO.
   *
   * \param ctx              The context to which this renderer
   *                             belongs.
   * \param target               The target framebuffer objects.
   * \param application_fps      The amount of fps of the application.
   * \param rendering_fps        The amount of fps of the rendering loop.
   */
  void render_outlined(RenderContext const& ctx,
                       FrameBufferObject const& fbo,
                       std::string const& text,
                       math::vec2i const& position,
                       utils::Color3f const& font_color,
                       utils::Color3f const& outline_color) const;

  void render(RenderContext const& ctx,
              FrameBufferObject const& fbo,
              std::string const& text,
              math::vec2i const& position,
              utils::Color3f const& font_color) const;

  static std::vector<std::string> get_system_font_directories();

 private:
  scm::gl::text_renderer_ptr text_renderer_;
  scm::gl::text_ptr text_;
};

}

#endif  // GUA_TEXT_RENDERER_HPP
