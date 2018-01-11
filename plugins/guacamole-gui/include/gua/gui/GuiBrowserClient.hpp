
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
#include <gua/gui/GLSurface.inl>
#include <include/wrapper/cef_message_router.h>


namespace gua{

class GuiBrowserClient :	public CefClient,
							//public CefDisplayHandler,
			               	public CefLifeSpanHandler,
			              	public CefRequestHandler
{
public:
    GuiBrowserClient(GLSurface *renderHandler, CefString url)
        : startup_url_(url)
        , m_renderHandler(renderHandler)
    {std::cout << "Its-a-me-Client!" << std::endl;}

    CefRefPtr<CefRenderHandler> GetRenderHandler() OVERRIDE {
        return m_renderHandler;
    }

    // CefClient methods:
	CefRefPtr<CefLifeSpanHandler> GetLifeSpanHandler() OVERRIDE { return this; }
	CefRefPtr<CefRequestHandler> GetRequestHandler() OVERRIDE { return this; }

	
	bool OnProcessMessageReceived(CefRefPtr<CefBrowser> browser,
	                            CefProcessId source_process,
	                            CefRefPtr<CefProcessMessage> message) OVERRIDE;

	// CefLifeSpanHandler methods:
	void OnAfterCreated(CefRefPtr<CefBrowser> browser) OVERRIDE;
	void OnBeforeClose(CefRefPtr<CefBrowser> browser) OVERRIDE;

	// CefRequestHandler methods:
	
	bool OnBeforeBrowse(CefRefPtr<CefBrowser> browser,
	                  CefRefPtr<CefFrame> frame,
	                  CefRefPtr<CefRequest> request,
	                  bool is_redirect) OVERRIDE;
	void OnRenderProcessTerminated(CefRefPtr<CefBrowser> browser,
	                             TerminationStatus status) OVERRIDE;

	
	const CefString startup_url_;

    CefRefPtr<CefRenderHandler> m_renderHandler;
     // Handles the browser side of query routing.
	CefRefPtr<CefMessageRouterBrowserSide> message_router_;
	scoped_ptr<CefMessageRouterBrowserSide::Handler> message_handler_;

    IMPLEMENT_REFCOUNTING(GuiBrowserClient);
    DISALLOW_COPY_AND_ASSIGN(GuiBrowserClient);
};

}

#endif //GUA_CEF_CLIENT_HPP

