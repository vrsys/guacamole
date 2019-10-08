////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of Swift2D.                                              //
//                                                                            //
// Copyright: (c) 2011-2014 Simon Schneegans & Felix Lauer                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

class AweProcessListener : public Awesomium::WebViewListener::Process
{
    ///////////////////////////////////////////////////////////////////////////////
    // ----------------------------------------------------------- public interface
  public:
    // ------------------------------------------------------------ public methods
    void OnUnresponsive(Awesomium::WebView* caller) { Logger::LOG_WARNING << "OnUnresponsive" << std::endl; }

    void OnResponsive(Awesomium::WebView* caller) { Logger::LOG_WARNING << "OnResponsive" << std::endl; }

    void OnCrashed(Awesomium::WebView* caller, Awesomium::TerminationStatus status) { Logger::LOG_WARNING << "OnCrashed" << std::endl; }

    void OnLaunch(Awesomium::WebView* caller)
    {
        // Logger::LOG_WARNING << "OnLaunch" << std::endl;
    }
};
