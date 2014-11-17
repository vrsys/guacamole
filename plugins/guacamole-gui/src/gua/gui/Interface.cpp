////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of Swift2D.                                              //
//                                                                            //
// Copyright: (c) 2011-2014 Simon Schneegans & Felix Lauer                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// includes  -------------------------------------------------------------------
#include <gua/gui/Interface.hpp>
#include <gua/gui/Paths.hpp>

#include <gua/utils/Logger.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/TextFile.hpp>

#include <Awesomium/WebCore.h>
#include <Awesomium/BitmapSurface.h>
#include <Awesomium/STLHelpers.h>

#include <mutex>

// Awesomium bug in linux
#ifndef _WIN32
Awesomium::DataSource::~DataSource(){}
#endif

namespace gua {

namespace {

#include "GLSurface.ipp"
#include "GLSurfaceFactory.ipp"
#include "AweDataSource.ipp"

}

////////////////////////////////////////////////////////////////////////////////

void Interface::update() const {
  web_core_->Update();
}

////////////////////////////////////////////////////////////////////////////////

Interface::Interface() {
  web_core_ = Awesomium::WebCore::Initialize(Awesomium::WebConfig());
  web_core_->set_surface_factory(new GLSurfaceFactory());

  Awesomium::WebPreferences prefs;
  prefs.enable_smooth_scrolling = true;
  web_session_ = web_core_->CreateWebSession(Awesomium::WSLit(""), prefs);

  Awesomium::DataSource* data_source = new AweDataSource();
  web_session_->AddDataSource(Awesomium::WSLit("gua"), data_source);
}

////////////////////////////////////////////////////////////////////////////////

Interface::~Interface() {
  auto factory = static_cast<GLSurfaceFactory*>(web_core_->surface_factory());
  Awesomium::WebCore::Shutdown();
  delete factory;
  web_session_->Release();
}

////////////////////////////////////////////////////////////////////////////////

bool Interface::bind(Awesomium::WebView* view, RenderContext const& ctx) const {
  auto surface = static_cast<GLSurface*>(view->surface());
  return surface && surface->bind(ctx);
}

////////////////////////////////////////////////////////////////////////////////

Awesomium::WebView* Interface::create_webview(int width, int height) const {
  return web_core_->CreateWebView(width, height, web_session_,
                                  Awesomium::kWebViewType_Offscreen);
}

////////////////////////////////////////////////////////////////////////////////

}

