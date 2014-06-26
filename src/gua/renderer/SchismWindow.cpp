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
#include <gua/renderer/SchismWindow.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

// external headers
#include <sstream>
#include <iostream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

SchismWindow::SchismWindow(Configuration const& configuration)
    : Window(configuration) {}

////////////////////////////////////////////////////////////////////////////////

SchismWindow::~SchismWindow() {

  close();
}

////////////////////////////////////////////////////////////////////////////////

void SchismWindow::open() {

  if (ctx_.window) {
    ctx_.window->hide();
  }

  ctx_.context.reset();
  ctx_.display.reset();
  ctx_.window.reset();

  scm::gl::wm::surface::format_desc window_format(
      scm::gl::FORMAT_RGBA_8, scm::gl::FORMAT_D24_S8, true, false);

  scm::gl::wm::context::attribute_desc context_attribs(
      4, 3, false, config.get_debug(), false);

  ctx_.display =
      scm::gl::wm::display_ptr(new scm::gl::wm::display(config.get_display_name()));

  ctx_.window = scm::gl::wm::window_ptr(new scm::gl::wm::window(
      ctx_.display,
      0,
      config.get_title(),
      math::vec2i(0, 0),
      math::vec2ui(config.get_size().x, config.get_size().y),
      window_format));

  ctx_.context = scm::gl::wm::context_ptr(
      new scm::gl::wm::context(ctx_.window, context_attribs));

  ctx_.window->show();

  Window::open();
}

////////////////////////////////////////////////////////////////////////////////

bool SchismWindow::get_is_open() const {
  return ctx_.window != nullptr;
}

////////////////////////////////////////////////////////////////////////////////

void SchismWindow::close() {
  if (get_is_open()) {
    ctx_.window->hide();
  }
}

////////////////////////////////////////////////////////////////////////////////

void SchismWindow::finish_frame() const {
  set_active(true);
  ctx_.window->swap_buffers(config.get_enable_vsync());
}

////////////////////////////////////////////////////////////////////////////////

}
