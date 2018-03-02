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
#include <gua/gui/GuiResource.hpp>

// guacamole headers
#include <gua/gui/Interface.hpp>
#include <gua/gui/GuiTexture.hpp>
#include <gua/gui/Paths.hpp>

namespace {
#include "GuiCefKeyEvent.ipp"
}

namespace gua {


////////////////////////////////////////////////////////////////////////////////

GuiResource::GuiResource()
  : name_("")
  , url_("")
  , mouse_position_()
  , gui_texture_(nullptr)
  //, view_(nullptr)
  //, js_window_(nullptr)
  , browser_()
  , browserClient_()
  , window_info_()
  , browserSettings_()
  , interactive_(true)
{}

void GuiResource::init(std::string const& name, std::string const& url,
                       math::vec2 const& size) {

  name_ = name;

  set_url(url);

  GLSurface* surface = new GLSurface(size.x, size.y);
  browserClient_ = new GuiBrowserClient(surface, &on_javascript_callback, &on_loaded);

  browser_ = Interface::instance()->create_browser(window_info_, browserClient_, url_, browserSettings_);
  std::cout << "Browser setup" << std::endl;
  gui_texture_ = std::make_shared<GuiTexture>(size.x, size.y, browserClient_);

  gua::TextureDatabase::instance()->add(name, gui_texture_);
  std::cout << "texture setup" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

GuiResource::~GuiResource() {}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::set_url(std::string const& url) {
  url_ = url;
  
  if(url_.find("asset://gua/") == 0) {
    url_.erase(0, 12);
    url_ = "file://" + gua::Paths::instance()->make_absolute(url_);
  }

}

////////////////////////////////////////////////////////////////////////////////

std::string const& GuiResource::get_url() const {
  return url_;
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::go_forward() {
  browser_->GoForward();
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::go_back() {
  browser_->GoBack();
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::go_to_history_offset(int offset) {
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::reload() {
  browser_->Reload();
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::focus() {
  browser_->GetHost()->SendFocusEvent(true);;
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::set_interactive(bool interactive) {
  interactive_ = interactive;
}

////////////////////////////////////////////////////////////////////////////////

bool GuiResource::is_interactive() const {
  return interactive_;
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::inject_keyboard_event(Key key, int scancode, int action, int mods) const {
  if (interactive_) {
    auto evt = static_cast<CefKeyEvent>(GuiCefKeyEvent(key, scancode, action, mods));
    browser_->GetHost()->SendKeyEvent(evt);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::inject_char_event(unsigned c) const {
  if (interactive_) {
    browser_->GetHost()->SendKeyEvent(GuiCefKeyEvent(c));
  }
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::inject_mouse_position_relative(math::vec2 const& position) {
  focus();
  inject_mouse_position(math::vec2(gui_texture_->width() * position.x, gui_texture_->height() * (1-position.y)));
}


////////////////////////////////////////////////////////////////////////////////

void GuiResource::inject_mouse_position(math::vec2 const& position) {
  if (interactive_) {
    mouse_position_ = position;
    CefMouseEvent evt;
    evt.x = position.x;
    evt.y = position.y;

    browser_->GetHost()->SendMouseMoveEvent(evt, false);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::inject_mouse_button(Button button, int action, int mods) const {
  if (interactive_) {
    bool up = (action == 0); 
    CefMouseEvent evt;
    evt.x = mouse_position_.x;
    evt.y = mouse_position_.y;

    browser_->GetHost()->SendMouseClickEvent(evt, MBT_LEFT, up, 1);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::inject_mouse_wheel(math::vec2 const& direction) const {
  if (interactive_) {
    CefMouseEvent evt;
    evt.x = mouse_position_.x;
    evt.y = mouse_position_.y;

    browser_->GetHost()->SendMouseWheelEvent(evt, direction.x * 20, direction.y*20);
  }
  
}

////////////////////////////////////////////////////////////////////////////////


void GuiResource::call_javascript_arg_vector(std::string const& method, std::vector<std::string> const& args) const {
  browserClient_->call_javascript(method, args);
}

////////////////////////////////////////////////////////////////////////////////
//
void GuiResource::call_javascript_arg_vector_async(std::string const& method, std::vector<std::string> const& args) const {
  /*
  if (!js_window_) {
    return;
  }

  Awesomium::JSArray j_args;
  for (auto const& arg: args) {
    j_args.Push(Awesomium::JSValue(Awesomium::ToWebString(arg)));
  }
  js_window_->ToObject().InvokeAsync(Awesomium::ToWebString(method), j_args);
  */
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::add_javascript_callback(std::string const& name) {
  add_javascript_callback(name, false);
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::add_javascript_getter(std::string const& name, std::function<std::string()> callback) {
  /*
  add_javascript_callback(name, true);
  result_callbacks_[name] = callback;
  */
}

////////////////////////////////////////////////////////////////////////////////

void GuiResource::add_javascript_callback(std::string const& callback, bool with_result) {
  /*
  Awesomium::JSValue o = view_->ExecuteJavascriptWithResult(
    Awesomium::WSLit("gua"), Awesomium::WSLit("")
  );

  if (o.IsObject()) {
    o.ToObject().SetCustomMethod(Awesomium::ToWebString(callback), with_result);
  }
  */
}

////////////////////////////////////////////////////////////////////////////////



}
