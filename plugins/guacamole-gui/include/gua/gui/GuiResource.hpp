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

#ifndef GUA_GUI_RESOURCE_HPP
#define GUA_GUI_RESOURCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/events/Signal.hpp>
#include <gua/gui/keyboard_enums.hpp>
#include <gua/gui/mouse_enums.hpp>
#include <gua/gui/stl_helpers.hpp>

namespace Awesomium
{
class WebView;
class JSValue;
} // namespace Awesomium

namespace gua
{
class GuiTexture;
class RenderContext;
class ShaderProgram;

/**
 * Stores geometry data.
 *
 * A mesh can be loaded from an Assimp mesh and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */
class GuiResource
{
  public:
    /**
     * Default constructor.
     *
     * Creates a new and empty Mesh.
     */
    GuiResource();
    ~GuiResource();

    void init(std::string const& name, std::string const& url = "", math::vec2 const& size = math::vec2(1000.f, 1000.f));

    events::Signal<std::string, std::vector<std::string>> on_javascript_callback;
    events::Signal<> on_loaded;

    void set_url(std::string const& url);
    std::string const& get_url() const;

    void go_forward();
    void go_back();
    void go_to_history_offset(int offset);

    void reload();
    void focus();

    void set_interactive(bool interactive);
    bool is_interactive() const;

    void inject_keyboard_event(Key key, int scancode, int action, int mods) const;
    void inject_char_event(unsigned c) const;

    void inject_mouse_position_relative(math::vec2 const& position) const;
    void inject_mouse_position(math::vec2 const& position) const;
    void inject_mouse_button(Button button, int action, int mods) const;
    void inject_mouse_wheel(math::vec2 const& direction) const;

    template <typename... Args>
    void call_javascript(std::string const& method, Args&&... a) const
    {
        std::vector<std::string> args = {(gua::to_string(a))...};
        call_javascript_arg_vector(method, args);
    }

    void call_javascript_arg_vector(std::string const& method, std::vector<std::string> const& args) const;

    template <typename... Args>
    void call_javascript_async(std::string const& method, Args&&... a) const
    {
        std::vector<std::string> args = {(gua::to_string(a))...};
        call_javascript_arg_vector_async(method, args);
    }

    void call_javascript_arg_vector_async(std::string const& method, std::vector<std::string> const& args) const;

    void add_javascript_callback(std::string const& name);
    void add_javascript_getter(std::string const& name, std::function<std::string()> callback);

    std::unordered_map<std::string, std::function<std::string()>> const& get_result_callbacks() const { return result_callbacks_; }

  private:
    void add_javascript_callback(std::string const& callback, bool with_result);

    std::string name_;
    std::string url_;

    std::shared_ptr<GuiTexture> gui_texture_ = nullptr;

    std::unordered_map<std::string, std::function<std::string()>> result_callbacks_;
    Awesomium::WebView* view_ = nullptr;
    Awesomium::JSValue* js_window_ = nullptr;
    bool interactive_ = true;
};

} // namespace gua

#endif // GUA_GUI_RESOURCE_HPP
