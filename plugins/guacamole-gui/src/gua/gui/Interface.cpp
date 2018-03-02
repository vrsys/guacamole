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

#include <include/wrapper/cef_message_router.h>

#include <Awesomium/WebCore.h>
#include <Awesomium/STLHelpers.h>

#include <gua/gui/GLSurface.hpp>

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

void Interface::update() {
  CefDoMessageLoopWork();
}


int Interface::init(int argc, char** argv) {
  // Provide CEF with command-line arguments.
  CefMainArgs main_args(argc, argv);

  // Create a temporary CommandLine object.
  CefRefPtr<CefCommandLine> command_line = CreateCommandLine(main_args);
  command_line->AppendSwitch("off-screen-rendering-enabled");

  // Create a CefApp of the correct processc type.
  CefRefPtr<CefApp> app;
  switch (GetProcessType(command_line)) {
    case PROCESS_TYPE_BROWSER:
      app = CreateBrowserProcessApp();
      break;
    case PROCESS_TYPE_RENDERER:
      app = CreateRendererProcessApp();
      break;
    case PROCESS_TYPE_OTHER:
      app = CreateOtherProcessApp();
      break;
  }

  // CEF applications have multiple sub-processes (render, plugin, GPU, etc)
  // that share the same executable. This function checks the command-line and,
  // if this is a sub-process, executes the appropriate logic.
  int exit_code = CefExecuteProcess(main_args, app, NULL);
  if (exit_code >= 0) {
    // The sub-process has completed so return here.
    return exit_code;
  }

  // Specify CEF global settings here.
  CefSettings settings;
  //CefString(&settings.resources_dir_path) = "./CEF-binaries/";
  //CefString(&settings.locales_dir_path) = "./CEF-binaries/";
  //CefString(&settings.log_file) = "./CEF-binaries/debug.log";

  settings.no_sandbox = true;

  // Initialize CEF for the browser process. The first browser instance will be
  // created in CefBrowserProcessHandler::OnContextInitialized() after CEF has
  // been initialized.
  CefInitialize(main_args, settings, app, NULL);

  return 0;
}

////////////////////////////////////////////////////////////////////////////////

Interface::Interface() {

}


Interface::~Interface() {
  std::cout << "Interface destroyed" << std::endl;
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
  std::cout << "it's a me, OTHER!" << std::endl;
  return NULL;
}

//TODO: building fails when returning RendererApp
CefRefPtr<CefApp> Interface::CreateRendererProcessApp() {
  std::cout << "it's a me, RENDERER!" << std::endl;
  return new gua::GuiRendererApp();
}


CefRefPtr<CefApp> Interface::CreateBrowserProcessApp() {
  std::cout << "it's a me, BOWSER!" << std::endl;
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

