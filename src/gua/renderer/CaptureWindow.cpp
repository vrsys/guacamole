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
#include <gua/renderer/CaptureWindow.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

// external headers
#include <scm/gl_core/window_management/window.h>

#include <sstream>
#include <iostream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

CaptureWindow::CaptureWindow(Configuration const& configuration)
  : WindowBase(configuration) {}

////////////////////////////////////////////////////////////////////////////////

CaptureWindow::~CaptureWindow() {

  close();
}

////////////////////////////////////////////////////////////////////////////////

void CaptureWindow::open() {

  if (window_) {
    window_->hide();
  }

  ctx_.context.reset();
  ctx_.display.reset();
  window_.reset();

  scm::gl::wm::surface::format_desc window_format(
      scm::gl::FORMAT_RGBA_8, scm::gl::FORMAT_D24_S8, true, false);

  scm::gl::wm::context::attribute_desc context_attribs(
      4, 4, false, config.get_debug(), false);

  ctx_.display =
      scm::gl::wm::display_ptr(new scm::gl::wm::display(config.get_display_name()));

  window_ = scm::gl::wm::window_ptr(new scm::gl::wm::window(
      ctx_.display,
      0,
      config.get_title(),
      math::vec2i(0, 0),
      math::vec2ui(config.get_size().x, config.get_size().y),
      window_format));

  ctx_.context = scm::gl::wm::context_ptr(
      new scm::gl::wm::context(window_, context_attribs));

  window_->show();
}

////////////////////////////////////////////////////////////////////////////////

bool CaptureWindow::get_is_open() const {
  return window_ != nullptr;
}

////////////////////////////////////////////////////////////////////////////////

bool CaptureWindow::should_close() const {
  return false;
}

////////////////////////////////////////////////////////////////////////////////

void CaptureWindow::close() {
  if (get_is_open()) {
    window_->hide();
  }
}

////////////////////////////////////////////////////////////////////////////////

void CaptureWindow::set_active(bool active) {
  ctx_.context->make_current(window_, active);
  if (!ctx_.render_device) {
    init_context();
  }
}

////////////////////////////////////////////////////////////////////////////////

void CaptureWindow::finish_frame() const {
  window_->swap_buffers(config.get_enable_vsync());
}

void CaptureWindow::set_image(RepImage img) {
  boost::upgrade_lock<boost::shared_mutex> lock(mutex_);
  boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
  image_ = img;
  std::cout << "set_image\n";
}

CaptureWindow::RepImage CaptureWindow::get_image() {
  boost::shared_lock<boost::shared_mutex> lock(mutex_);
  return image_;
}
////////////////////////////////////////////////////////////////////////////////

}
