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

// external headers

namespace gua {
namespace gui {

struct RenderContext;

/**
 * Stores geometry data.
 *
 * A mesh can be loaded from an Assimp mesh and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */
class GuiResource : public GeometryResource {
  public:

    /**
     * Default constructor.
     *
     * Creates a new and empty Mesh.
     */
    GuiResource(std::string const& url = "");
    ~GuiResource();

    events::Signal<std::string, std::vector<std::string>> on_javascript_callback;
    events::Signal<>                                      on_loaded;

    void set_url(std::string const& url);
    std::string const& get_url() const;

    void reload();
    void focus();

    void set_interactive(bool interactive);
    bool is_interactive() const;

    void inject_keyboard_event(Key key, int scancode, int action, int mods) const;
    void inject_char_event(unsigned c) const;

    void inject_mouse_position(math::vec2 const& position) const;
    void inject_mouse_button(Button button, int action, int mods) const;
    void inject_mouse_wheel(math::vec2 const& direction) const;

    template<typename ...Args>
    void call_javascript(std::string const& method, Args&& ... a) const {
      std::vector<std::string> args = {(to_string(a))...};
      call_javascript_impl(method, args);
    }

    void add_javascript_callback(std::string const& name);
    void add_javascript_getter(std::string const& name, std::function<std::string()> callback);



    void bind(RenderContext const& context, ShaderProgram* program) const;

    /*virtual*/ void ray_test(Ray const& ray, PickResult::Options options,
                  node::Node* owner, std::set<PickResult>& hits);


  private:

    void call_javascript_impl(std::string const& method, std::vector<std::string> const& args) const;
    void add_javascript_callback(std::string const& callback, bool with_result);

    std::string url_;

    std::unordered_map<std::string, std::function<std::string()>> result_callbacks_;
    Awesomium::WebView* view_;
    Awesomium::JSValue* js_window_;
    bool                interactive_;

};

}
}

#endif  // GUA_GUI_RESOURCE_HPP
