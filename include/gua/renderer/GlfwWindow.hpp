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

#ifndef GUA_GLFW_WINDOW_HPP
#define GUA_GLFW_WINDOW_HPP

// guacamole headers
#include <gua/renderer/WindowBase.hpp>
#include <gua/events.hpp>

struct GLFWwindow;

namespace gua
{
/**
 * A window for displaying stuff.
 *
 * It's a window which can display OpenGL stuff.
 */
class GUA_DLL GlfwWindow : public WindowBase
{
  public:
    /**
     * Constructor.
     *
     * Creates a new Window. It owns a RenderContext where Geomtries
     * can be drawn to.
     *
     * \param description   The description of the window.
     */
    GlfwWindow(Configuration const& configuration = Configuration());

    /**
     * Destructor.
     *
     * Cleans all associated memory.
     */
    virtual ~GlfwWindow();

    void open() override;
    bool should_close() const override;
    bool get_is_open() const override;
    void close() override;
    void process_events() override;

    /**
     * Control the cursor mode.
     *
     * NORMAL - usual cursor
     * HIDDEN - cursor is invisible while within window
     * DISABLED - cursor is hidden and captured in window until it is tabbed
     */
    enum class CursorMode
    {
        NORMAL = 0,
        HIDDEN = 1,
        DISABLED = 2
    };

    void cursor_mode(CursorMode mode);
    CursorMode cursor_mode() const;

    /**
     * Activate the context of this window.
     *
     * Makes the RenderContext of this window current. All preceeding
     * OpenGL calls will be invoked on this window.
     */
    void set_active(bool active);

    events::Signal<math::vec2ui> on_resize;
    events::Signal<int, int, int, int> on_key_press; // int key, int scancode, int action, int mods
    events::Signal<unsigned> on_char;
    events::Signal<int, int, int> on_button_press; // int button, int action, int mods
    events::Signal<math::vec2> on_move_cursor;
    events::Signal<math::vec2> on_scroll;
    events::Signal<bool> on_enter;

    friend class Pipeline;

  private:
    GLFWwindow* glfw_window_;

    CursorMode cursor_mode_;

    void swap_buffers_impl() override;
};

} // namespace gua

#endif // GUA_GLFW_WINDOW_HPP
