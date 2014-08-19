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
#include <gua/renderer/nurbs_geometry/TrimmedBezierSurfaceObject.hpp>

// header, system
#include <boost/bind.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/TrimDomainSerializer_BinaryContourMap.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurfaceObject::TrimmedBezierSurfaceObject() : _surfaces() {}

////////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurfaceObject::~TrimmedBezierSurfaceObject() {}

////////////////////////////////////////////////////////////////////////////////
void TrimmedBezierSurfaceObject::add(surface_ptr const& surface) {
  _surfaces.insert(surface);
}

////////////////////////////////////////////////////////////////////////////////
void TrimmedBezierSurfaceObject::remove(surface_ptr const& surface) {
  _surfaces.erase(surface);
}

////////////////////////////////////////////////////////////////////////////////
void TrimmedBezierSurfaceObject::print(std::ostream& os) const {
  os << std::endl << "Bezierobject : " << std::endl;
  std::for_each(_surfaces.begin(),
                _surfaces.end(),
                boost::bind(&surface_type::print, _1, boost::ref(os)));
}

////////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurfaceObject::bbox_t TrimmedBezierSurfaceObject::bbox() const {
  double xmin = std::numeric_limits<double>::max();
  double xmax = -std::numeric_limits<double>::max();
  double ymin = std::numeric_limits<double>::max();
  double ymax = -std::numeric_limits<double>::max();
  double zmin = std::numeric_limits<double>::max();
  double zmax = -std::numeric_limits<double>::max();

  BOOST_FOREACH(surface_ptr const & surface, _surfaces) {
    bbox_t tmp = surface->bbox();
    xmax = std::max(xmax, tmp.max[0]);
    ymax = std::max(ymax, tmp.max[1]);
    zmax = std::max(zmax, tmp.max[2]);
    xmin = std::min(xmin, tmp.min[0]);
    ymin = std::min(ymin, tmp.min[1]);
    zmin = std::min(zmin, tmp.min[2]);
  }

  return bbox_t(tml::point3d(xmin, ymin, zmin), tml::point3d(xmax, ymax, zmax));
}

////////////////////////////////////////////////////////////////////////////////
std::size_t TrimmedBezierSurfaceObject::size() const {
  return _surfaces.size();
}

////////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurfaceObject::const_surface_iterator
TrimmedBezierSurfaceObject::begin() const {
  return _surfaces.begin();
}

////////////////////////////////////////////////////////////////////////////////
TrimmedBezierSurfaceObject::const_surface_iterator
TrimmedBezierSurfaceObject::end() const {
  return _surfaces.end();
}

}  // namespace gua
