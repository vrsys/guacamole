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
#include <gua/renderer/TextRenderer.hpp>

// guacamole headers
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/math/math.hpp>

#include <boost/filesystem.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

TextRenderer::TextRenderer(RenderContext const& ctx,
                           float size,
                           std::string const& font_name) {
  try {
    std::string path_to_font;

    for (auto const& font_dir: get_system_font_directories()) {
      std::string font_path_candidate = font_dir + "/" + font_name;
      if (boost::filesystem::exists(font_path_candidate)) {
        path_to_font = font_path_candidate;
      }
    }

    scm::gl::font_face_ptr font(
        new scm::gl::font_face(ctx.render_device,
                               path_to_font,
                               size,
                               0.7f,
                               scm::gl::font_face::smooth_normal));

    text_renderer_.reset(new scm::gl::text_renderer(ctx.render_device));

    text_.reset(new scm::gl::text(
        ctx.render_device, font, scm::gl::font_face::style_regular, ""));

    text_->text_kerning(true);
  }
  catch (std::exception & e) {
    throw std::runtime_error(std::string("Could not load font. ") + e.what());
  }

}

////////////////////////////////////////////////////////////////////////////////

void TextRenderer::render_outlined(RenderContext const& ctx,
                                   FrameBufferObject const& fbo,
                                   std::string const& text,
                                   math::vec2i const& position,
                                   utils::Color3f const& font_color,
                                   utils::Color3f const& outline_color) const {

  text_->text_color(
      math::vec4(font_color.r(), font_color.g(), font_color.b(), 1.0f));
  text_->text_outline_color(math::vec4(
      outline_color.r(), outline_color.g(), outline_color.b(), 1.0f));

  math::mat4 fs_projection =
      scm::math::make_ortho_matrix(0.0f,
                                   static_cast<float>(fbo.width()),
                                   0.0f,
                                   static_cast<float>(fbo.height()),
                                   -1.0f,
                                   1.0f);

  text_renderer_->projection_matrix(fs_projection);
  text_->text_string(text);
  text_renderer_->draw_outlined(ctx.render_context, position, text_);
}

////////////////////////////////////////////////////////////////////////////////

void TextRenderer::render(RenderContext const& ctx,
                          FrameBufferObject const& fbo,
                          std::string const& text,
                          math::vec2i const& position,
                          utils::Color3f const& font_color) const {

  text_->text_color(
      math::vec4(font_color.r(), font_color.g(), font_color.b(), 1.0f));

  math::mat4 fs_projection =
      scm::math::make_ortho_matrix(0.0f,
                                   static_cast<float>(fbo.width()),
                                   0.0f,
                                   static_cast<float>(fbo.height()),
                                   -1.0f,
                                   1.0f);

  text_renderer_->projection_matrix(fs_projection);
  text_->text_string(text);
  text_renderer_->draw(ctx.render_context, position, text_);
}

////////////////////////////////////////////////////////////////////////////////

/* static */ std::vector<std::string>
TextRenderer::get_system_font_directories() {
  std::vector<std::string> path_to_fonts;
  path_to_fonts.push_back(".");
#ifdef _WIN32
  // windows implementation
  path_to_fonts.push_back(std::string(std::getenv("WINDIR")) + "/Fonts");
#else
#ifdef _APPLE_
  // macos implementation
  path_to_fonts.push_back("/Library/Fonts");
  path_to_fonts.push_back("/System Folder/Fonts");
  path_to_fonts.push_back("/System/Library/Fonts");
  path_to_fonts.push_back("/Network/Library/Fonts");
  path_to_fonts.push_back("~/Library/Fonts");
#else
  // linux implementation
  path_to_fonts.push_back("/usr/share/fonts/truetype/freefont/");
  path_to_fonts.push_back("/usr/share/fonts");
  path_to_fonts.push_back("/usr/local/share/fonts");
  path_to_fonts.push_back("~/.fonts");
#endif
#endif
  return path_to_fonts;
}

}
