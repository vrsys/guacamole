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

#ifndef GUA_GUI_INTERFACE_HPP
#define GUA_GUI_INTERFACE_HPP

// includes  -------------------------------------------------------------------
#include <gua/utils/Singleton.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/events/Signal.hpp>
#include <gua/gui/mouse_enums.hpp>

#include <include/cef_app.h>
#include <include/cef_command_line.h>
#include <gua/gui/GuiBrowserClient.hpp>
/*
#if defined(OS_WIN)
#include <windows.h>
#endif
*/

// forward declares ------------------------------------------------------------

namespace gua {

class ShaderProgram;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// -----------------------------------------------------------------------------
class GUA_DLL Interface : public Singleton<Interface> {

 ///////////////////////////////////////////////////////////////////////////////
 // ----------------------------------------------------------- public interface
 public:
  void update();
  int init(int argc, char** argv);

  friend class GuiResource;
  friend class Singleton<Interface>;

  // Process types that may have different CefApp instances.
  enum ProcessType {
    PROCESS_TYPE_BROWSER,
    PROCESS_TYPE_RENDERER,
    PROCESS_TYPE_OTHER,
  };

 ///////////////////////////////////////////////////////////////////////////////
 // ---------------------------------------------------------- private interface
 private:
  // this class is a Singleton --- private c'tor and d'tor
  Interface();
  ~Interface();

  CefRefPtr<CefBrowser> create_browser(CefWindowInfo& info, CefRefPtr<GuiBrowserClient> client,
                                       std::string url, CefBrowserSettings settings) const;
  // Called in the main browser process to create the CefApp for that process.
  CefRefPtr<CefApp> CreateBrowserProcessApp();

  // Called in the renderer sub-process to create the CefApp for that process.
  CefRefPtr<CefApp> CreateRendererProcessApp();

  // Called in other sub-processes to create the CefApp for that process.
  CefRefPtr<CefApp> CreateOtherProcessApp();

  // Create a new CommandLine object for use before CEF initialization.
  CefRefPtr<CefCommandLine> CreateCommandLine(const CefMainArgs& main_args);
  // Determine the process type based on command-line arguments.
  ProcessType GetProcessType(const CefRefPtr<CefCommandLine>& command_line);

 ///////////////////////////////////////////////////////////////////////////////
 // ---------------------------------------------------------- private member

  //process flags
  const std::string kProcessType = "type";
  const std::string kRendererProcess = "renderer";
  #if defined(OS_LINUX)
  const std::string kZygoteProcess = "zygote";
  #endif
};

// -----------------------------------------------------------------------------

}

#endif // GUA_GUI_INTERFACE_HPP
