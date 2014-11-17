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
#include <gua/gui/GuiTexture.hpp>

#include <gua/utils/Logger.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/TextFile.hpp>

#include <Awesomium/WebCore.h>
#include <Awesomium/STLHelpers.h>

#include <mutex>

#include "GLSurface.inl"

// Awesomium bug in linux
#ifndef _WIN32
Awesomium::DataSource::~DataSource(){}
#endif

namespace gua {

namespace {

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

Awesomium::WebView* Interface::create_webview(int width, int height) const {
  return web_core_->CreateWebView(width, height, web_session_,
                                  Awesomium::kWebViewType_Offscreen);
}

////////////////////////////////////////////////////////////////////////////////

}

