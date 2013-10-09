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

#ifndef GUA_TRIMMEDNURBSSURFACEOBJECT_HPP
#define GUA_TRIMMEDNURBSSURFACEOBJECT_HPP

#ifdef _MSC_VER
#pragma warning(disable : 4251)
#endif

// header, system
#include <vector>

// header, external

// header, project
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurface.hpp>

namespace gua {

// simple container for all nurbssurfaces of Object
class TrimmedNurbsSurfaceObject {
 public:  // enums, typedefs

  typedef TrimmedNurbsSurface surface_type;
  typedef std::vector<surface_type> surface_container;
  typedef surface_type::curve_type curve_type;

  typedef surface_type::curve_iterator curve_iterator;
  typedef surface_type::const_curve_iterator const_curve_iterator;

  typedef surface_container::iterator iterator;
  typedef surface_container::const_iterator const_iterator;

 public:  // c'tor / d'tor

  TrimmedNurbsSurfaceObject();
  ~TrimmedNurbsSurfaceObject();

 public:  // methods

  // add a rational surface to scene
  void add(surface_type const& nrbs);

  // print
  void print(std::ostream& os) const;

  const_iterator begin() const;
  const_iterator end() const;

  std::size_t surfaces() const;
  std::size_t trimcurves() const;

 private:  // data members

  surface_container _surfaces;
};

}  // namespace gua

#endif  // GUA_TRIMMEDNURBSSURFACEOBJECT_HPP
