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
#include <gua/renderer/nurbs_geometry/TrimmedBezierSurface.hpp>

// header, system

// header, project

using namespace tml;

namespace gua {

//////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurface::TrimmedBezierSurface()
    : beziersurface3d(),
      _trimdomain(),
      _bezierdomain(curve_point_type(0, 0), curve_point_type(1, 1)) {}

//////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurface::TrimmedBezierSurface(
    beziersurface3d const& untrimmed_surface)
    : beziersurface3d(untrimmed_surface),
      _trimdomain(),
      _bezierdomain(curve_point_type(0, 0), curve_point_type(1, 1)) {}

//////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurface::TrimmedBezierSurface(TrimmedBezierSurface const& bs)
    : beziersurface3d(bs),
      _trimdomain(bs._trimdomain),
      _bezierdomain(bs._bezierdomain) {}

//////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurface::~TrimmedBezierSurface() {}

//////////////////////////////////////////////////////////////////////////////
void TrimmedBezierSurface::swap(TrimmedBezierSurface& other) {
  beziersurface3d::swap(other);

  std::swap(_trimdomain, other._trimdomain);
  std::swap(_bezierdomain, other._bezierdomain);
}

//////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurface& TrimmedBezierSurface::operator=(
    TrimmedBezierSurface const& cpy) {
  TrimmedBezierSurface tmp(cpy);
  swap(tmp);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////
void TrimmedBezierSurface::trimtype(bool t) { _trimdomain->type(t); }

//////////////////////////////////////////////////////////////////////////////
void TrimmedBezierSurface::nurbsdomain(TrimDomain::bbox_type const& n) {
  _trimdomain->nurbsdomain(n);
}

//////////////////////////////////////////////////////////////////////////////
void TrimmedBezierSurface::bezierdomain(TrimDomain::bbox_type const& b) {
  _bezierdomain = b;
}

//////////////////////////////////////////////////////////////////////////////
TrimDomain::bbox_type const& TrimmedBezierSurface::bezierdomain() const {
  return _bezierdomain;
}

//////////////////////////////////////////////////////////////////////////////
bool TrimmedBezierSurface::trimtype() const { return _trimdomain->type(); }

//////////////////////////////////////////////////////////////////////////////
std::size_t TrimmedBezierSurface::trimcurves() const {
  return _trimdomain->size();
}

//////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurface::trimdomain_ptr const&
TrimmedBezierSurface::domain() const {
  return _trimdomain;
}

//////////////////////////////////////////////////////////////////////////////
void TrimmedBezierSurface::domain(trimdomain_ptr const& domain) {
  _trimdomain = domain;
}

//////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurface::mesh_type const& TrimmedBezierSurface::points() const {
  return _points;
}

//////////////////////////////////////////////////////////////////////////////
void TrimmedBezierSurface::print(std::ostream& os) const {
  beziersurface3d::print(os);

  os << "rational trimming curves : " << _trimdomain->size() << std::endl;

  _trimdomain->print(os);
}

}  // namespace gua
