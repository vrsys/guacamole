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
#include <gua/renderer/ScreenshotPass.hpp>
#include <gua/renderer/CaptureWindow.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <cstdint>
#include <FreeImagePlus.h>
#include <boost/make_shared.hpp>
#include <scm/gl_core/render_device/opengl/util/data_format_helper.h>

namespace gua {

namespace {

////////////////////////////////////////////////////////////////////////////////

void screenshot(PipelinePass& pass, PipelinePassDescription const& pass_desc, Pipeline& pipe) {
  auto const& ctx(pipe.get_context());

  auto color = pipe.get_gbuffer().get_current_color_buffer();
  if (!color)
    return;
  auto texture_ptr = color->get_buffer(ctx);
  scm::gl::texture_2d_ptr _color_buffer_resolved = boost::dynamic_pointer_cast<scm::gl::texture_2d>(texture_ptr);

  if (!_color_buffer_resolved)
    return;

  std::cout << "hey\n";

  gua::CaptureWindow::RepImage img;
  img.width = _color_buffer_resolved->descriptor()._size.x;
  img.height = _color_buffer_resolved->descriptor()._size.y;
  img.bpp = scm::gl::bit_per_pixel(_color_buffer_resolved->format());
  img.gl_type = scm::gl::util::gl_base_type(_color_buffer_resolved->format());
  img.gl_internal_format =
      scm::gl::util::gl_internal_format(_color_buffer_resolved->format());
  img.gl_base_format =
      scm::gl::util::gl_base_format(_color_buffer_resolved->format());

  int img_size = img.width * img.height *
                 scm::gl::size_of_format(_color_buffer_resolved->format());
  img.data = std::vector<char>(img_size);

  ctx.render_context->retrieve_texture_data(_color_buffer_resolved, 0, img.data.data());

  if (ctx.render_window) {
    CaptureWindow* w = (CaptureWindow*)(ctx.render_window);
    if (w) {
      w->set_image(img);
    }
  }
}

}

////////////////////////////////////////////////////////////////////////////////

ScreenshotPassDescription::ScreenshotPassDescription()
  : PipelinePassDescription() {
  needs_color_buffer_as_input_ = false; // don't ping pong the color buffer
  writes_only_color_buffer_ = false; // we write out a color
  doClear_ = false;
  rendermode_ = RenderMode::Custom;
  process_ = screenshot;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> ScreenshotPassDescription::make_copy() const {
  return std::make_shared<ScreenshotPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass ScreenshotPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
  PipelinePass pass{*this, ctx, substitution_map};
  return pass;
}

}
