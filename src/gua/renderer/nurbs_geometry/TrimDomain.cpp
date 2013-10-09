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

// header, i/f
#include <gua/renderer/nurbs_geometry/TrimDomain.hpp>

// header, system
#include <iterator>
#include <limits>
#include <boost/bind.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/bezierclipping2d.hpp>

using namespace tml;

namespace gua {

/////////////////////////////////////////////////////////////////////////////
TrimDomain::TrimDomain()
    : _trimloops(),
      _type(false),
      _nurbsdomain(point_type(0, 0), point_type(1, 1)) {}

/////////////////////////////////////////////////////////////////////////////
TrimDomain::~TrimDomain() {}

/////////////////////////////////////////////////////////////////////////////
void TrimDomain::swap(TrimDomain& swp) {
  std::swap(_trimloops, swp._trimloops);
  std::swap(_type, swp._type);
  std::swap(_nurbsdomain, swp._nurbsdomain);
}

/////////////////////////////////////////////////////////////////////////////
void TrimDomain::add(contour_type const& bc) { _trimloops.push_back(bc); }

/////////////////////////////////////////////////////////////////////////////
std::size_t TrimDomain::size() const {
  std::size_t curves = 0;
  for (auto l = _trimloops.begin(); l != _trimloops.end(); ++l) {
    curves += l->size();
  }
  return curves;
}

/////////////////////////////////////////////////////////////////////////////
bool TrimDomain::empty() const { return _trimloops.empty(); }

/////////////////////////////////////////////////////////////////////////////
void TrimDomain::nurbsdomain(bbox_type const& n) { _nurbsdomain = n; }

/////////////////////////////////////////////////////////////////////////////
TrimDomain::bbox_type const& TrimDomain::nurbsdomain() const {
  return _nurbsdomain;
}

/////////////////////////////////////////////////////////////////////////////
bool TrimDomain::type() const { return _type; }

/////////////////////////////////////////////////////////////////////////////
void TrimDomain::type(bool inner) { _type = inner; }

/////////////////////////////////////////////////////////////////////////////
TrimDomain::curve_container TrimDomain::curves() const {
  curve_container curves;
  BOOST_FOREACH(contour_type const & loop, _trimloops) {
    std::copy(loop.begin(), loop.end(), std::back_inserter(curves));
  }
  return curves;
}

/////////////////////////////////////////////////////////////////////////////
std::size_t TrimDomain::loop_count() const { return _trimloops.size(); }

/////////////////////////////////////////////////////////////////////////////
TrimDomain::trimloop_container const& TrimDomain::loops() const {
  return _trimloops;
}

/////////////////////////////////////////////////////////////////////////////
void TrimDomain::print(std::ostream& os) const {
  std::vector<curve_ptr> all_curves = curves();

  os << "nurbs domain total range : " << _nurbsdomain << std::endl;
  os << "# trimming curves : " << all_curves.size() << std::endl;
  std::for_each(all_curves.begin(),
                all_curves.end(),
                boost::bind(&beziercurve2d::print, _1, boost::ref(os), ""));
}

/////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& os, TrimDomain const& t) {
  t.print(os);
  return os;
}

}  // namespace gua
