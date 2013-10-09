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

#ifndef GUA_TRIMMED_NURBSSURFACE_HPP
#define GUA_TRIMMED_NURBSSURFACE_HPP

// header, system
#include <vector>

// header external
#include <gua/renderer/nurbs_geometry/tml/parametric/nurbscurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/nurbssurface.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

// header, project

namespace gua {

class TrimmedNurbsSurface : public tml::nurbssurface<tml::point3d> {
 public:  // typedefs, enums

  typedef double value_type;
  typedef tml::point<value_type, 3> point_type;
  typedef tml::nurbssurface<point_type> surface_type;
  typedef tml::nurbscurve<tml::point<value_type, 2> > curve_type;

  typedef std::vector<curve_type> curve_container;
  typedef curve_container::iterator curve_iterator;
  typedef curve_container::const_iterator const_curve_iterator;

  typedef std::vector<curve_container> trimloop_container;
  typedef trimloop_container::iterator trimloop_iterator;
  typedef trimloop_container::const_iterator const_trimloop_iterator;

 public:  // c'tor / d'tor

  TrimmedNurbsSurface();
  TrimmedNurbsSurface(TrimmedNurbsSurface const& rhs);
  ~TrimmedNurbsSurface();

  void swap(TrimmedNurbsSurface& rhs);
  TrimmedNurbsSurface& operator=(TrimmedNurbsSurface const& rhs);

 public:  // methods

  void add(curve_container const& tc);

  trimloop_container const& trimloops() const;

  void print(std::ostream& os) const;

  void trimtype(bool type);
  bool trimtype() const;

 private:  // data members

  trimloop_container _trimloops;
  bool _trimtype;  // 0 = trim inner, 1 = trim outer
};

std::ostream& operator<<(std::ostream& os, TrimmedNurbsSurface const& rhs);

}  // namespace gua

#endif  // GUA_TRIMMED_NURBSSURFACE_HPP
