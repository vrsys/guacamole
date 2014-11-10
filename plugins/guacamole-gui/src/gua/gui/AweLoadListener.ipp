////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of Swift2D.                                              //
//                                                                            //
// Copyright: (c) 2011-2014 Simon Schneegans & Felix Lauer                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

class AweLoadListener : public Awesomium::WebViewListener::Load {

 ///////////////////////////////////////////////////////////////////////////////
 // ----------------------------------------------------------- public interface
 public:

  // ----------------------------------------------------- contruction interface
  AweLoadListener(GuiComponent* parent)
    : parent_(parent) {}

  // ------------------------------------------------------------ public methods
  void OnBeginLoadingFrame(
    Awesomium::WebView* caller, int64 frame_id,
    bool is_main_frame, const Awesomium::WebURL& url,
    bool is_error_page) {}

  void OnFailLoadingFrame(
    Awesomium::WebView* caller, int64 frame_id,
    bool is_main_frame, const Awesomium::WebURL& url,
    int error_code, const Awesomium::WebString& error_desc) {}

  void OnFinishLoadingFrame(
    Awesomium::WebView* caller, int64 frame_id,
    bool is_main_frame, const Awesomium::WebURL& url) {}

  void OnDocumentReady(
    Awesomium::WebView* caller, const Awesomium::WebURL& url) {

    caller->CreateGlobalJavascriptObject(Awesomium::WSLit("Swift2D"));
    parent_->on_loaded.emit();
  }

 ///////////////////////////////////////////////////////////////////////////////
 // ---------------------------------------------------------- private interface
 private:
  GuiComponent* parent_;
};
