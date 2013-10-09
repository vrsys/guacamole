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

// class header
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurface.hpp>

// header, system
#include <cassert>  // std::assert

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/nurbscurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/nurbssurface.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>

namespace gua {

//////////////////////////////////////////////////////////////////////////////
TrimmedNurbsSurface::TrimmedNurbsSurface()
    : tml::nurbssurface<tml::point3d>(), _trimloops(), _trimtype(true) {}

//////////////////////////////////////////////////////////////////////////////
TrimmedNurbsSurface::TrimmedNurbsSurface(TrimmedNurbsSurface const& rhs)
    : tml::nurbssurface<tml::point3d>(rhs),
      _trimloops(rhs._trimloops),
      _trimtype(rhs._trimtype) {}

//////////////////////////////////////////////////////////////////////////////
TrimmedNurbsSurface::~TrimmedNurbsSurface() {}

//////////////////////////////////////////////////////////////////////////////
void TrimmedNurbsSurface::swap(TrimmedNurbsSurface& rhs) {
  tml::nurbssurface<tml::point3d>::swap(rhs);

  std::swap(_trimloops, rhs._trimloops);
  std::swap(_trimtype, rhs._trimtype);

}

//////////////////////////////////////////////////////////////////////////////
TrimmedNurbsSurface& TrimmedNurbsSurface::operator=(
    TrimmedNurbsSurface const& rhs) {
  TrimmedNurbsSurface tmp(rhs);
  swap(tmp);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////
void TrimmedNurbsSurface::add(curve_container const& tc) {
  _trimloops.push_back(tc);
}

//////////////////////////////////////////////////////////////////////////////
TrimmedNurbsSurface::trimloop_container const&
TrimmedNurbsSurface::trimloops() const {
  return _trimloops;
}

//////////////////////////////////////////////////////////////////////////////
void TrimmedNurbsSurface::print(std::ostream& os) const {
  tml::nurbssurface3d::print(os);

  os << "number of trimloops : " << _trimloops.size() << std::endl;

  os << "trimcurves : " << std::endl;

  BOOST_FOREACH(curve_container const & loop, _trimloops) {
    BOOST_FOREACH(curve_type const & curve, loop) { curve.print(os); }
  }

}

//////////////////////////////////////////////////////////////////////////////
void TrimmedNurbsSurface::trimtype(bool type) { _trimtype = type; }

//////////////////////////////////////////////////////////////////////////////
bool TrimmedNurbsSurface::trimtype() const { return _trimtype; }

//////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& os, TrimmedNurbsSurface const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace gua
