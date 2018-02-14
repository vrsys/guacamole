
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

GuiMessageHandler::GuiMessageHandler(const CefString& startup_url)
    : startup_url_(startup_url) {}

void GuiMessageHandler::set_message(CefString message){
  std::unique_lock<std::mutex> lock(mutex_);
  message_ = message;
}

void GuiMessageHandler::send(){
  //std::cout << "manual callback!\n";
  
  if(callback_ == nullptr) return;

  std::unique_lock<std::mutex> lock(mutex_);
  callback_->Success(message_);

}

///////////////////////////////////////////////////////////////////////////
// Called due to cefQuery execution in message_router.html.
bool GuiMessageHandler::OnQuery(CefRefPtr<CefBrowser> browser,
             CefRefPtr<CefFrame> frame,
             int64 query_id,
             const CefString& request,
             bool persistent,
             CefRefPtr<Callback> callback) {

  if(persistent) callback_ = callback;
  send();

  return true;
}

} //namespace gua
