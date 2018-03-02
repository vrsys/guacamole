
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


#ifndef GUA_CEF_CLIENT_HPP
#define GUA_CEF_CLIENT_HPP

#include <include/cef_client.h>
#include <include/cef_render_handler.h>
#include <include/cef_life_span_handler.h>
#include <include/cef_request_handler.h>
#include <include/wrapper/cef_message_router.h>
#include <gua/gui/GLSurface.hpp>
#include <gua/gui/GuiMessageHandler.hpp>



namespace gua{

class GuiBrowserClient :	public CefClient,
			               	public CefLifeSpanHandler,
			              	public CefRequestHandler,
			              	public CefKeyboardHandler
{
public:
    GuiBrowserClient(GLSurface *renderHandler,
					 events::Signal<std::string, std::vector<std::string>>* on_js_callback,
					 events::Signal<>* on_loaded);

    //JS communication
    void set_message(CefString message);
    void send_message();

	void call_javascript(std::string functionName, std::vector<std::string> const& args);

	///////////////////////////////////////////////////////////////////////////
	// CefClient methods:
    CefRefPtr<CefRenderHandler> GetRenderHandler() OVERRIDE { return m_renderHandler; }
	CefRefPtr<CefLifeSpanHandler> GetLifeSpanHandler() OVERRIDE { return this; }
	CefRefPtr<CefRequestHandler> GetRequestHandler() OVERRIDE { return this; }

	
	bool OnProcessMessageReceived(CefRefPtr<CefBrowser> browser,
	                            CefProcessId source_process,
	                            CefRefPtr<CefProcessMessage> message) OVERRIDE;
	///////////////////////////////////////////////////////////////////////////
	// CefLifeSpanHandler methods:
	void OnAfterCreated(CefRefPtr<CefBrowser> browser) OVERRIDE;
	void OnBeforeClose(CefRefPtr<CefBrowser> browser) OVERRIDE;
	///////////////////////////////////////////////////////////////////////////
	// CefRequestHandler methods:	
	bool OnBeforeBrowse(CefRefPtr<CefBrowser> browser,
	                  CefRefPtr<CefFrame> frame,
	                  CefRefPtr<CefRequest> request,
	                  bool is_redirect) OVERRIDE;
	void OnRenderProcessTerminated(CefRefPtr<CefBrowser> browser,
	                             TerminationStatus status) OVERRIDE;
	///////////////////////////////////////////////////////////////////////////
	//CefKeyboardHandler methods:
	bool OnPreKeyEvent(CefRefPtr<CefBrowser> browser,
                             const CefKeyEvent& event,
                             CefEventHandle os_event,
                             bool* is_keyboard_shortcut) OVERRIDE;
	///////////////////////////////////////////////////////////////////////////

	private:
    CefRefPtr<CefRenderHandler> m_renderHandler;
     // Handles the browser side of query routing.
	CefRefPtr<CefMessageRouterBrowserSide> message_router_;
	scoped_ptr<gua::GuiMessageHandler> message_handler_;

	events::Signal<std::string, std::vector<std::string>>* on_javascript_callback_;
	events::Signal<>*                                      on_loaded_;

    IMPLEMENT_REFCOUNTING(GuiBrowserClient);
    DISALLOW_COPY_AND_ASSIGN(GuiBrowserClient);
};

}

#endif //GUA_CEF_CLIENT_HPP

