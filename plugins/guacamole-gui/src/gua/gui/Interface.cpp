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

// includes  -------------------------------------------------------------------
#include <gua/gui/Interface.hpp>
#include <gua/gui/Paths.hpp>
#include <gua/gui/GuiTexture.hpp>

#include <gua/utils/Logger.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/TextFile.hpp>

#include <Awesomium/WebCore.h>
#include <Awesomium/STLHelpers.h>

#include <gua/gui/GLSurface.inl>

// Awesomium bug in linux
#ifndef _WIN32
Awesomium::DataSource::~DataSource(){}
#endif

namespace gua {

namespace {

#include "GLSurfaceFactory.ipp"
#include "AweDataSource.ipp"
#include "GuiProcessApps.ipp"

}

////////////////////////////////////////////////////////////////////////////////

void Interface::update() const {
  CefDoMessageLoopWork();
}


int Interface::init(int argc, char** argv) const{
  //parses CEF command line arguments (if there are any)
  CefMainArgs args(argc, argv);

    {
        int result = CefExecuteProcess(args, nullptr, nullptr);
        // checkout CefApp, derive it and set it as second parameter, for more control on
        // command args and resources.
        if (result >= 0) // child proccess has endend, so exit.
        {
            return result;
        }
        if (result == -1)
        {
            // we are here in the father proccess.
        }
    }
    {
        bool result = CefInitialize(args, settings_, nullptr, nullptr);
        // CefInitialize creates a sub-proccess and executes the same executeable, as calling CefInitialize, if not set different in settings.browser_subprocess_path
        // if you create an extra program just for the childproccess you only have to call CefExecuteProcess(...) in it.
        if (!result)
        {
            // handle error(g_fullscreen_textures[0]
            return -1;
        }
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

Interface::Interface() {
  /*
  web_core_ = Awesomium::WebCore::Initialize(Awesomium::WebConfig());
  web_core_->set_surface_factory(new GLSurfaceFactory());

  Awesomium::WebPreferences prefs;
  prefs.enable_smooth_scrolling = true;
  web_session_ = web_core_->CreateWebSession(Awesomium::WSLit(""), prefs);

  Awesomium::DataSource* data_source = new AweDataSource();
  web_session_->AddDataSource(Awesomium::WSLit("gua"), data_source);
  */
}


Interface::~Interface() {
  std::cout << "Interface destroyed" << std::endl;
  /*
  auto factory = static_cast<GLSurfaceFactory*>(web_core_->surface_factory());
  Awesomium::WebCore::Shutdown();
  delete factory;
  web_session_->Release();
  */
  CefShutdown();
}

////////////////////////////////////////////////////////////////////////////////

CefRefPtr<CefBrowser> Interface::create_browser(CefWindowInfo& info, CefRefPtr<GuiBrowserClient> client,
                                       std::string url, CefBrowserSettings settings) const{
  info.SetAsWindowless(0);
  return CefBrowserHost::CreateBrowserSync(info, client.get(), url, settings, nullptr);
}

// No CefApp for other subprocesses.
CefRefPtr<CefApp> Interface::CreateOtherProcessApp() {
  return NULL;
}

//TODO: building fails when returning RendererApp
CefRefPtr<CefApp> Interface::CreateRendererProcessApp() {
  //return NULL;
  return new gua::GuiRendererApp();
}


CefRefPtr<CefApp> Interface::CreateBrowserProcessApp() {
  return new gua::GuiBrowserApp();
}

////////////////////////////////////////////////////////////////////////////////

CefRefPtr<CefCommandLine> Interface::CreateCommandLine(const CefMainArgs& main_args) {
  CefRefPtr<CefCommandLine> command_line = CefCommandLine::CreateCommandLine();
  #if defined(OS_WIN)
    command_line->InitFromString(::GetCommandLineW());
  #else
    command_line->InitFromArgv(main_args.argc, main_args.argv);
  #endif
  return command_line;
}

Interface::ProcessType Interface::GetProcessType(const CefRefPtr<CefCommandLine>& command_line) {
  // The command-line flag won't be specified for the browser process.
  if (!command_line->HasSwitch(kProcessType))
    return PROCESS_TYPE_BROWSER;

  const std::string& process_type = command_line->GetSwitchValue(kProcessType);
  if (process_type == kRendererProcess)
    return PROCESS_TYPE_RENDERER;

  #if defined(OS_LINUX)
    // On Linux the zygote process is used to spawn other process types. Since we
    // don't know what type of process it will be we give it the renderer app.
    if (process_type == kZygoteProcess)
      return PROCESS_TYPE_RENDERER;
  #endif

  return PROCESS_TYPE_OTHER;
}
////////////////////////////////////////////////////////////////////////////////
} //namespace gua

