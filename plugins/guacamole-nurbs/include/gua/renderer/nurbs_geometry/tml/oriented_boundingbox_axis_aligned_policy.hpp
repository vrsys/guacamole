/********************************************************************************
*
* Copyright (C) 2011 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : oriented_boundingbox_axis_aligned_policy.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_ORIENTED_BOUNDINGBOX_AXIS_ALIGNED_POLICY_HPP
#define TML_ORIENTED_BOUNDINGBOX_AXIS_ALIGNED_POLICY_HPP

// header, system
#include <list>
#include <boost/array.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/matrix.hpp>

namespace tml {

template <typename point_t> class axis_aligned_policy {
 public:  // typedefs

  static unsigned const N = point_t::size;

  typedef typename point_t::value_type value_type;
  typedef point_t point_type;
  typedef matrix<value_type, N, N> matrix_type;

 public:

  //////////////////////////////////////////////////////////////////////////////
  axis_aligned_policy(pointmesh2d<point_t> const& mesh) {}

  //////////////////////////////////////////////////////////////////////////////
  axis_aligned_policy(pointmesh3d<point_t> const& mesh) {}

  //////////////////////////////////////////////////////////////////////////////
  axis_aligned_policy() {}

  //////////////////////////////////////////////////////////////////////////////
  template <typename any_type> void apply(any_type) { return; }

  //////////////////////////////////////////////////////////////////////////////
  template <typename iterator_t>
  void operator()(iterator_t begin,
                  iterator_t end,
                  point_t& obb_center,
                  matrix_type& obb_orientation,
                  point_t& obb_low,
                  point_t& obb_high) {
    obb_center = mean(begin, end);
    obb_orientation = orientation(begin, end, obb_center);
    limits(begin, end, obb_center, obb_orientation, obb_low, obb_high);
  }

  //////////////////////////////////////////////////////////////////////////////
  template <typename iterator_t>
  point_t mean(iterator_t point_begin, iterator_t point_end) const {
    point_t sum = std::accumulate(point_begin, point_end, point_t());
    return sum / value_type(std::distance(point_begin, point_end));
  }

  //////////////////////////////////////////////////////////////////////////////
  template <typename iterator_t>
  matrix_type orientation(iterator_t point_begin,
                          iterator_t point_end,
                          point_t const& center) const {
    return matrix_type();
  }

  //////////////////////////////////////////////////////////////////////////////
  template <typename iterator_t>
  void limits(iterator_t begin,
              iterator_t end,
              point_t& obb_center,
              matrix_type const& obb_orientation,
              point_t& obb_low,
              point_t& obb_high) const {
    typename oriented_boundingbox<point_t>::matrix_type inv =
        obb_orientation.inverse();

    // preset limits to min/max for local coordinates and global coordinates (
    // to determine center )
    obb_low = point_t::maximum();
    obb_high = point_t::minimum();

    point_t g_low = point_t::maximum();
    point_t g_high = point_t::minimum();

    while (begin != end) {
      // project point into normalized bounding box space
      point_t p = inv * ((*begin) - obb_center);

      for (unsigned i = 0; i != point_t::size; ++i) {
        obb_low[i] = std::min(obb_low[i], p[i]);
        obb_high[i] = std::max(obb_high[i], p[i]);
      }

      ++begin;
    }

    point_t mean = (obb_low + obb_high) / value_type(2);
    point_t dist = obb_high - obb_low;

    point_t delta = obb_orientation * (mean);

    obb_low = dist / value_type(-2);
    obb_high = dist / value_type(2);

    obb_center += delta;
  }

 private:

};

}  // namespace tml

#endif  // TML_ORIENTED_BOUNDINGBOX_AXIS_ALIGNED_POLICY_HPP
