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

#ifndef GUA_TRIMDOMAIN_HPP
#define GUA_TRIMDOMAIN_HPP

#ifdef _MSC_VER
#pragma warning(disable : 4251)
#endif

// header, system
#include <vector>
#include <iostream>

// header, external
#include <boost/shared_ptr.hpp>

#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziercurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour_segment.hpp>

// header, project

namespace gua {

class TrimDomain {
 public:  // enums/typedefs

  typedef double value_type;
  typedef tml::point<double, 2> point_type;

  typedef tml::contour_segment<value_type> contour_segment_type;
  typedef boost::shared_ptr<contour_segment_type> contour_segment_ptr;
  typedef boost::shared_ptr<TrimDomain> domain_ptr;

  typedef tml::beziercurve<point_type> curve_type;
  typedef boost::shared_ptr<curve_type> curve_ptr;
  typedef std::vector<curve_ptr> curve_container;

  typedef tml::axis_aligned_boundingbox<point_type> bbox_type;
  typedef tml::contour<value_type> contour_type;

  typedef std::vector<contour_type> trimloop_container;

 public:  // c'tor/d'tor

  TrimDomain();
  ~TrimDomain();

  void swap(TrimDomain& swp);

 public:  // methods

  // add loop
  void add(contour_type const& loop);

  std::size_t size() const;

  bool empty() const;

  void nurbsdomain(bbox_type const&);
  bbox_type const& nurbsdomain() const;

  // true = trim outer values; false = trim inner values
  bool type() const;
  void type(bool inner);

  curve_container curves() const;
  std::size_t loop_count() const;
  trimloop_container const& loops() const;

  void print(std::ostream& os) const;

 private:  // member

  // curves and type(inner outer trim)
  trimloop_container _trimloops;
  bool _type;
  bbox_type _nurbsdomain;  // [umin, umax, vmin, vmax]
};

std::ostream& operator<<(std::ostream& os, TrimDomain const& t);

}  // namespace gua

#endif  // GUA_TRIMDOMAIN_HPP
