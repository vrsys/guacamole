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

#include "GLSurface.inl"

// Awesomium bug in linux
#ifndef _WIN32
Awesomium::DataSource::~DataSource() {}
#endif

namespace gua
{
namespace
{
#include "GLSurfaceFactory.ipp"
#include "AweDataSource.ipp"

} // namespace

////////////////////////////////////////////////////////////////////////////////

void Interface::update() const { web_core_->Update(); }

////////////////////////////////////////////////////////////////////////////////

Interface::Interface()
{
    web_core_ = Awesomium::WebCore::Initialize(Awesomium::WebConfig());
    web_core_->set_surface_factory(new GLSurfaceFactory());

    Awesomium::WebPreferences prefs;
    prefs.enable_smooth_scrolling = true;
    web_session_ = web_core_->CreateWebSession(Awesomium::WSLit(""), prefs);

    Awesomium::DataSource* data_source = new AweDataSource();
    web_session_->AddDataSource(Awesomium::WSLit("gua"), data_source);
}

////////////////////////////////////////////////////////////////////////////////

Interface::~Interface()
{
    auto factory = static_cast<GLSurfaceFactory*>(web_core_->surface_factory());
    Awesomium::WebCore::Shutdown();
    delete factory;
    web_session_->Release();
}

////////////////////////////////////////////////////////////////////////////////

Awesomium::WebView* Interface::create_webview(int width, int height) const { return web_core_->CreateWebView(width, height, web_session_, Awesomium::kWebViewType_Offscreen); }

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
