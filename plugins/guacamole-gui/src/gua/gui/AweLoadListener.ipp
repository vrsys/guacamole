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

class AweLoadListener : public Awesomium::WebViewListener::Load
{
    ///////////////////////////////////////////////////////////////////////////////
    // ----------------------------------------------------------- public interface
  public:
    // ----------------------------------------------------- contruction interface
    AweLoadListener(GuiResource* parent) : parent_(parent) {}

    // ------------------------------------------------------------ public methods
    void OnBeginLoadingFrame(Awesomium::WebView* caller, int64 frame_id, bool is_main_frame, const Awesomium::WebURL& url, bool is_error_page) {}

    void OnFailLoadingFrame(Awesomium::WebView* caller, int64 frame_id, bool is_main_frame, const Awesomium::WebURL& url, int error_code, const Awesomium::WebString& error_desc) {}

    void OnFinishLoadingFrame(Awesomium::WebView* caller, int64 frame_id, bool is_main_frame, const Awesomium::WebURL& url) {}

    void OnDocumentReady(Awesomium::WebView* caller, const Awesomium::WebURL& url)
    {
        caller->CreateGlobalJavascriptObject(Awesomium::WSLit("gua"));
        parent_->on_loaded.emit();
    }

    ///////////////////////////////////////////////////////////////////////////////
    // ---------------------------------------------------------- private interface
  private:
    GuiResource* parent_;
};
