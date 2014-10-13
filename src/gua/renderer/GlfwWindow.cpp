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
#include <gua/renderer/GlfwWindow.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

// external headers
#include <sstream>
#include <iostream>
#include <GLFW/glfw3.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void on_window_resize(GLFWwindow* glfw_window, int width, int height) {
  auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));

  window->config.set_size(math::vec2ui(width, height));
  window->on_resize.emit(window->config.get_size());
}

////////////////////////////////////////////////////////////////////////////////
void on_window_key_press(GLFWwindow* glfw_window, int key, int scancode, int action, int mods) {
  auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
  window->on_key_press.emit(key, scancode, action, mods);
}

////////////////////////////////////////////////////////////////////////////////

void on_window_char(GLFWwindow* glfw_window, unsigned c) {
  auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
  window->on_char.emit(c);
}

////////////////////////////////////////////////////////////////////////////////

void on_window_button_press(GLFWwindow* glfw_window, int button, int action, int mods) {
  auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
  window->on_button_press.emit(button, action, mods);
}

////////////////////////////////////////////////////////////////////////////////

void on_window_move_cursor(GLFWwindow* glfw_window, double x, double y) {
  auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
  window->on_move_cursor.emit(math::vec2(x, y));
}

////////////////////////////////////////////////////////////////////////////////

void on_window_scroll(GLFWwindow* glfw_window, double x, double y) {
  auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
  window->on_scroll.emit(math::vec2(x, y));
}

////////////////////////////////////////////////////////////////////////////////

void on_window_enter(GLFWwindow* glfw_window, int enter) {
  auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
  window->on_enter.emit(enter);
}

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

GlfwWindow::GlfwWindow(Configuration const& configuration)
  : WindowBase(configuration),
    glfw_window_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

GlfwWindow::~GlfwWindow() {

  close();
}

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::open() {

  int monitor_count(0);
  auto monitors(glfwGetMonitors(&monitor_count));

  if (monitor_count == 0) {
    Logger::LOG_WARNING << "Failed to open GlfwWindow: No monitor found!" << std::endl;
    glfwTerminate();
    return;
  }

  if (config.monitor() >= monitor_count) {
    Logger::LOG_WARNING << "Failed to open GlfwWindow: There is no monitor with the number " << config.monitor() << "!" << std::endl;
    glfwTerminate();
    return;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, config.get_debug());

  glfw_window_ = glfwCreateWindow(
    config.get_size().x, config.get_size().y,
    config.get_title().c_str(), 
    config.get_fullscreen_mode()? glfwGetPrimaryMonitor(): nullptr, nullptr
  );

  glfwSetWindowUserPointer(glfw_window_, this);
  glfwSetWindowSizeCallback(glfw_window_, &on_window_resize);

  glfwSetKeyCallback(         glfw_window_, &on_window_key_press);
  glfwSetCharCallback(        glfw_window_, &on_window_char);
  glfwSetMouseButtonCallback( glfw_window_, &on_window_button_press);
  glfwSetCursorPosCallback(   glfw_window_, &on_window_move_cursor);
  glfwSetScrollCallback(      glfw_window_, &on_window_scroll);
  glfwSetCursorEnterCallback( glfw_window_, &on_window_enter);

  if (!glfw_window_) {
    Logger::LOG_WARNING << "Failed to open GlfwWindow: Could not create glfw3 window!" << std::endl;
    glfwTerminate();
    return;
  }

  WindowBase::open();
}

////////////////////////////////////////////////////////////////////////////////

bool GlfwWindow::get_is_open() const {
  return glfw_window_;
}

////////////////////////////////////////////////////////////////////////////////

bool GlfwWindow::should_close() const {
  return glfw_window_ && glfwWindowShouldClose(glfw_window_);
}

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::close() {
  if (get_is_open()) {
    glfwDestroyWindow(glfw_window_);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::process_events() {
  glfwPollEvents();
}

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::set_active(bool active) const {
  glfwMakeContextCurrent(glfw_window_);
}

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::finish_frame() const {

  set_active(true);

  glfwSwapInterval(config.get_enable_vsync()? 1 : 0);
  glfwSwapBuffers(glfw_window_);

  // Workaround for Windows Window Handling
  // Poll events from rendering thread and not application mainloop
  // Otherwise application window is stalling 
  glfwPollEvents();
}

////////////////////////////////////////////////////////////////////////////////

}
