/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : TrimmedNurbsSurfaceObject.cpp
*  project    : gua
*  description:
*
********************************************************************************/

// header i/f
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurfaceObject.hpp>

// header, system
#include <iostream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
TrimmedNurbsSurfaceObject::TrimmedNurbsSurfaceObject() : _surfaces() {}

////////////////////////////////////////////////////////////////////////////////
TrimmedNurbsSurfaceObject::~TrimmedNurbsSurfaceObject() {}

////////////////////////////////////////////////////////////////////////////////
void TrimmedNurbsSurfaceObject::add(TrimmedNurbsSurface const& nrbs) {
  _surfaces.push_back(nrbs);
}

////////////////////////////////////////////////////////////////////////////////
void TrimmedNurbsSurfaceObject::print(std::ostream& os) const {
  os << "Non-Rational NURBS in Object : " << std::endl;
  for (auto i = _surfaces.begin(); i < _surfaces.end(); ++i) {
    os << (*i);
  }
}

////////////////////////////////////////////////////////////////////////////////
TrimmedNurbsSurfaceObject::const_iterator
TrimmedNurbsSurfaceObject::begin() const {
  return _surfaces.begin();
}

////////////////////////////////////////////////////////////////////////////////
TrimmedNurbsSurfaceObject::const_iterator
TrimmedNurbsSurfaceObject::end() const {
  return _surfaces.end();
}

////////////////////////////////////////////////////////////////////////////////
std::size_t TrimmedNurbsSurfaceObject::surfaces() const {
  return _surfaces.size();
}

////////////////////////////////////////////////////////////////////////////////
std::size_t TrimmedNurbsSurfaceObject::trimcurves() const {
  std::size_t ntrimcurves = 0;

  for (auto i = _surfaces.begin(); i != _surfaces.end(); ++i) {
    for (auto loop = i->trimloops().begin(); loop != i->trimloops().end();
         ++loop) {
      ntrimcurves += std::size_t(loop->size());
    }
  }

  return ntrimcurves;
}

}  // namespace gua
