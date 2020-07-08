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

namespace gua
{
void error_callback(int error, const char* description) { throw std::runtime_error(description); }

////////////////////////////////////////////////////////////////////////////////

void on_window_resize(GLFWwindow* glfw_window, int width, int height)
{
    auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));

    window->config.set_size(math::vec2ui(width, height));
    window->on_resize.emit(window->config.get_size());
}

////////////////////////////////////////////////////////////////////////////////
void on_window_key_press(GLFWwindow* glfw_window, int key, int scancode, int action, int mods)
{
    auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
    window->on_key_press.emit(key, scancode, action, mods);
}

////////////////////////////////////////////////////////////////////////////////

void on_window_char(GLFWwindow* glfw_window, unsigned c)
{
    auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
    window->on_char.emit(c);
}

////////////////////////////////////////////////////////////////////////////////

void on_window_button_press(GLFWwindow* glfw_window, int button, int action, int mods)
{
    auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
    window->on_button_press.emit(button, action, mods);
}

////////////////////////////////////////////////////////////////////////////////

void on_window_move_cursor(GLFWwindow* glfw_window, double x, double y)
{
    auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
    window->on_move_cursor.emit(math::vec2(float(x), float(window->config.get_size().y - y)));
}

////////////////////////////////////////////////////////////////////////////////

void on_window_scroll(GLFWwindow* glfw_window, double x, double y)
{
    auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
    window->on_scroll.emit(math::vec2(float(x), float(y)));
}

////////////////////////////////////////////////////////////////////////////////

void on_window_enter(GLFWwindow* glfw_window, int enter)
{
    auto window(static_cast<GlfwWindow*>(glfwGetWindowUserPointer(glfw_window)));
    window->on_enter.emit(enter != 0);
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

GlfwWindow::GlfwWindow(Configuration const& configuration) : WindowBase(configuration), glfw_window_(nullptr), cursor_mode_{CursorMode::NORMAL} {}

////////////////////////////////////////////////////////////////////////////////

GlfwWindow::~GlfwWindow() { close(); }

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::open()
{
    glfwSetErrorCallback(error_callback);

    int monitor_count(0);
    auto monitors(glfwGetMonitors(&monitor_count));

    if(monitor_count == 0)
    {
        Logger::LOG_WARNING << "Failed to open GlfwWindow: No monitor found!" << std::endl;
        glfwTerminate();
        return;
    }

    if(config.monitor() >= monitor_count)
    {
        Logger::LOG_WARNING << "Failed to open GlfwWindow: There is no monitor with the number " << config.monitor() << "!" << std::endl;
        glfwTerminate();
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, config.get_debug());

    if(config.get_stereo_mode() == StereoMode::QUAD_BUFFERED)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }

    glfw_window_ = glfwCreateWindow(config.get_size().x, config.get_size().y, config.get_title().c_str(), config.get_fullscreen_mode() ? glfwGetPrimaryMonitor() : nullptr, nullptr);
    if(!glfw_window_)
    {
        throw std::runtime_error("GlfwWindow::open() : unable to create window");
    }

    glfwSetWindowUserPointer(glfw_window_, this);
    glfwSetWindowSizeCallback(glfw_window_, &on_window_resize);

    glfwSetKeyCallback(glfw_window_, &on_window_key_press);
    glfwSetCharCallback(glfw_window_, &on_window_char);
    glfwSetMouseButtonCallback(glfw_window_, &on_window_button_press);
    glfwSetCursorPosCallback(glfw_window_, &on_window_move_cursor);
    glfwSetScrollCallback(glfw_window_, &on_window_scroll);
    glfwSetCursorEnterCallback(glfw_window_, &on_window_enter);

    switch(cursor_mode_)
    {
    case CursorMode::NORMAL:
        glfwSetInputMode(glfw_window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        break;
    case CursorMode::HIDDEN:
        glfwSetInputMode(glfw_window_, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
        break;
    case CursorMode::DISABLED:
        glfwSetInputMode(glfw_window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        break;
    }

    if(!glfw_window_)
    {
        Logger::LOG_WARNING << "Failed to open GlfwWindow: Could not create glfw3 window!" << std::endl;
        glfwTerminate();
        return;
    }
}

////////////////////////////////////////////////////////////////////////////////

bool GlfwWindow::get_is_open() const { return glfw_window_ != 0; }

////////////////////////////////////////////////////////////////////////////////

bool GlfwWindow::should_close() const { return glfw_window_ && glfwWindowShouldClose(glfw_window_); }

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::close()
{
    if(get_is_open())
    {
        glfwDestroyWindow(glfw_window_);
        glfw_window_ = nullptr;
    }
}

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::process_events() { glfwPollEvents(); }

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::cursor_mode(CursorMode mode)
{
    switch(mode)
    {
    case CursorMode::NORMAL:
        if(get_is_open())
            glfwSetInputMode(glfw_window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        cursor_mode_ = mode;
        break;
    case CursorMode::HIDDEN:
        if(get_is_open())
            glfwSetInputMode(glfw_window_, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
        cursor_mode_ = mode;
        break;
    case CursorMode::DISABLED:
        if(get_is_open())
            glfwSetInputMode(glfw_window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        cursor_mode_ = mode;
        break;
    default:
        Logger::LOG_WARNING << "Cursor mode undefined or unsupported" << std::endl;
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////

GlfwWindow::CursorMode GlfwWindow::cursor_mode() const { return cursor_mode_; }

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::set_active(bool active)
{
    glfwMakeContextCurrent(glfw_window_);
    if(!ctx_.render_device)
    {
        init_context();
    }
}

////////////////////////////////////////////////////////////////////////////////

void GlfwWindow::swap_buffers_impl()
{
    glfwSwapInterval(config.get_enable_vsync() ? 1 : 0);
    glfwSwapBuffers(glfw_window_);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
