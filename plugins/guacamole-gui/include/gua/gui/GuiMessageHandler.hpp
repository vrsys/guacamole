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
#ifndef GUA_GUI_MESSAGE_HANDLER_
#define GUA_GUI_MESSAGE_HANDLER_

#include <include/wrapper/cef_message_router.h>
#include <include/wrapper/cef_helpers.h>
#include <gua/gui/stl_helpers.hpp>
#include <mutex>


namespace gua {

// Handle messages in the browser process.
class GuiMessageHandler : public CefMessageRouterBrowserSide::Handler {
 public:
  GuiMessageHandler(const CefString& startup_url);

  void send();
  void set_message(CefString message);

  ///////////////////////////////////////////////////////////////////////////
  // Called due to cefQuery execution in message_router.html.
  bool OnQuery(CefRefPtr<CefBrowser> browser,
               CefRefPtr<CefFrame> frame,
               int64 query_id,
               const CefString& request,
               bool persistent,
               CefRefPtr<Callback> callback) OVERRIDE;

 private:
  const CefString startup_url_;
  CefString message_;
  CefRefPtr<Callback> callback_;

  std::mutex mutex_;

  DISALLOW_COPY_AND_ASSIGN(GuiMessageHandler);
};

} //namespace gua

#endif
