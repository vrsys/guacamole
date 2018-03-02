
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
#include <gua/gui/GuiMessageHandler.hpp>


namespace gua {

GuiMessageHandler::GuiMessageHandler(events::Signal<std::string, std::vector<std::string>>* on_js_callback,
                                      events::Signal<>* on_loaded)
  : on_javascript_callback_(on_js_callback)
  , on_loaded_(on_loaded){
    fastwriter_["indentation"] = "";
}

std::string GuiMessageHandler::create_function_call(std::string functionName, std::vector<std::string> const& args){
  //create JSON function object
  Json::Value function;
  function["functionName"] = functionName;

  for(int i = 0; i < args.size(); ++i){
    function["parameters"][i] = args[i];
  }

  return Json::writeString(fastwriter_, function);
}

bool GuiMessageHandler::call_javascript(std::string call) {
  if(callback_ == nullptr) return false;

  std::unique_lock<std::mutex> lock(mutex_);
  callback_->Success(call);

  return true;
}

/*
std::vector<std::string> GuiMessageHandler::split(std::string s, std::string delimiter){
  size_t pos = 0;
  std::string token;
  std::vector<std::string> list;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    list.push_back(token);
    std::cout << token << std::endl;
    s.erase(0, pos + delimiter.length());
  }
}
 */

std::vector<std::string> GuiMessageHandler::split(std::string str, char delimiter){
      std::istringstream f(str);
      std::string s;
      std::vector<std::string> list;
      while (getline(f, s, delimiter)) {
        list.push_back(s);
      }

      return list;
}

///////////////////////////////////////////////////////////////////////////
// Called due to cefQuery execution in message_router.html.
bool GuiMessageHandler::OnQuery(CefRefPtr<CefBrowser> browser,
             CefRefPtr<CefFrame> frame,
             int64 query_id,
             const CefString& request,
             bool persistent,
             CefRefPtr<Callback> callback) {

  std::string requestString = request;

  if(requestString.find(callback_string_) == 0){
    std::string method;
    std::vector<std::string> args;
    requestString.erase(0, 11);
    args = split(requestString, '.');
    method = args.front();
    args.erase(args.begin());

    on_javascript_callback_->emit(method, args);

  }
  if(persistent) callback_ = callback;
  //callback->Success("");

  return true;
}

} //namespace gua
