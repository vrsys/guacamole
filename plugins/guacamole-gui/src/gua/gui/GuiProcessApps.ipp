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
#include <include/cef_app.h>
#include <include/cef_browser_process_handler.h>
#include <include/cef_render_process_handler.h>
#include <include/wrapper/cef_message_router.h>

namespace gua {

// Implementation of CefApp for the browser process.
class GuiBrowserApp : public CefApp, public CefBrowserProcessHandler {
 public:
  GuiBrowserApp() {}

  // CefApp methods:
  CefRefPtr<CefBrowserProcessHandler> GetBrowserProcessHandler() OVERRIDE {
    return this;
  }

 private:
  IMPLEMENT_REFCOUNTING(GuiBrowserApp);
  DISALLOW_COPY_AND_ASSIGN(GuiBrowserApp);
};



// Implementation of CefApp for the renderer process.
class GuiRendererApp : public CefApp, public CefRenderProcessHandler {
 public:
  GuiRendererApp() {}

  // CefApp methods:
  CefRefPtr<CefRenderProcessHandler> GetRenderProcessHandler() OVERRIDE {
    return this;
  }

  // CefRenderProcessHandler methods:
  void OnWebKitInitialized() OVERRIDE {
    // Create the renderer-side router for query handling.
    CefMessageRouterConfig config;
    message_router_ = CefMessageRouterRendererSide::Create(config);
  }

  void OnContextCreated(CefRefPtr<CefBrowser> browser,
                        CefRefPtr<CefFrame> frame,
                        CefRefPtr<CefV8Context> context) OVERRIDE {
    message_router_->OnContextCreated(browser, frame, context);
  }

  void OnContextReleased(CefRefPtr<CefBrowser> browser,
                         CefRefPtr<CefFrame> frame,
                         CefRefPtr<CefV8Context> context) OVERRIDE {
    message_router_->OnContextReleased(browser, frame, context);
  }

  bool OnProcessMessageReceived(CefRefPtr<CefBrowser> browser,
                                CefProcessId source_process,
                                CefRefPtr<CefProcessMessage> message) OVERRIDE {
    return message_router_->OnProcessMessageReceived(browser, source_process,
                                                     message);
  }

 private:
  // Handles the renderer side of query routing.
  CefRefPtr<CefMessageRouterRendererSide> message_router_;

  IMPLEMENT_REFCOUNTING(GuiRendererApp);
  DISALLOW_COPY_AND_ASSIGN(GuiRendererApp);
};



} //namespace gua
