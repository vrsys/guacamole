
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

#include <gua/gui/GuiBrowserClient.hpp>
#include <include/wrapper/cef_helpers.h>

namespace gua {

//GuiBrowserClient
//////////////////////////////////////////////////////////////////////////
GuiBrowserClient::GuiBrowserClient(GLSurface *renderHandler, CefString url)
        : startup_url_(url)
        , m_renderHandler(renderHandler)
    {std::cout << "Its-a-me-Client!" << std::endl;}

void GuiBrowserClient::set_message(CefString message){
  message_ = message;
}

void GuiBrowserClient::send_message(){
  message_handler_->send(message_); 
}

bool GuiBrowserClient::OnProcessMessageReceived(CefRefPtr<CefBrowser> browser,
                                      CefProcessId source_process,
                                      CefRefPtr<CefProcessMessage> message) {

	
  CEF_REQUIRE_UI_THREAD();

  return message_router_->OnProcessMessageReceived(browser, source_process,
                                                   message);
}


void GuiBrowserClient::OnAfterCreated(CefRefPtr<CefBrowser> browser) {
  CEF_REQUIRE_UI_THREAD();

  if (!message_router_) {
    // Create the browser-side router for query handling.
    CefMessageRouterConfig config;
    message_router_ = CefMessageRouterBrowserSide::Create(config);

    // Register handlers with the router.
    message_handler_.reset(new GuiMessageHandler(startup_url_));
    message_router_->AddHandler(message_handler_.get(), false);
  }
}

void GuiBrowserClient::OnBeforeClose(CefRefPtr<CefBrowser> browser) {
  CEF_REQUIRE_UI_THREAD();
    // Free the router when the last browser is closed.
    message_router_->RemoveHandler(message_handler_.get());
    message_handler_.reset();
    message_router_ = NULL;
}

bool GuiBrowserClient::OnBeforeBrowse(CefRefPtr<CefBrowser> browser,
                            CefRefPtr<CefFrame> frame,
                            CefRefPtr<CefRequest> request,
                            bool is_redirect) {
  CEF_REQUIRE_UI_THREAD();

  message_router_->OnBeforeBrowse(browser, frame);
  return false;
}

void GuiBrowserClient::OnRenderProcessTerminated(CefRefPtr<CefBrowser> browser,
                                       TerminationStatus status) {
  CEF_REQUIRE_UI_THREAD();

  message_router_->OnRenderProcessTerminated(browser);
}

bool GuiBrowserClient::OnPreKeyEvent(CefRefPtr<CefBrowser> browser,
                             const CefKeyEvent& event,
                             CefEventHandle os_event,
                             bool* is_keyboard_shortcut){
  std::cout << "Event handled " << event.native_key_code << std::endl;
  return true;

}



}  // namespace gua

