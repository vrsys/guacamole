/********************************************************************************
*
* Copyright (C) 2011 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : oriented_boundingbox_random_policy.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_ORIENTED_BOUNDINGBOX_RANDOM_POLICY_HPP
#define TML_ORIENTED_BOUNDINGBOX_RANDOM_POLICY_HPP

// header, system
#include <list>
#include <ctime>
#include <boost/array.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/matrix.hpp>

namespace tml {

template <typename point_t> class random_policy {
 public:  // typedefs

  static unsigned const N = point_t::size;

  typedef typename point_t::value_type value_type;
  typedef point_t point_type;
  typedef matrix<value_type, N, N> matrix_type;

 public:

  //////////////////////////////////////////////////////////////////////////////
  random_policy(pointmesh2d<point_t> const& mesh, std::size_t iterations = 100)
      : _iterations(iterations) {}

  //////////////////////////////////////////////////////////////////////////////
  random_policy(pointmesh3d<point_t> const& mesh, std::size_t iterations = 100)
      : _iterations(iterations) {}

  //////////////////////////////////////////////////////////////////////////////
  random_policy(std::size_t iterations = 100) : _iterations(iterations) {}

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
                          point_t& center) const {
    matrix<value_type, N, N> orientation;
    value_type tightest_volume = boost::numeric::bounds<value_type>::highest();

    for (int k = 0; k != _iterations; ++k) {
      matrix<value_type, N, N> random;

      point_t dir1 = generate_random_point();
      point_t dir2 = generate_random_point();
      point_t dir3 = generate_random_point();

      // use ortho-normal system
      dir1 /= dir1.length();
      dir2 /= dir2.length();
      dir3 /= dir3.length();

      // normalize
      dir3 = cross(dir1, dir2);
      dir2 = cross(dir1, dir3);

      random[0][0] = dir1[0];
      random[1][0] = dir1[1];
      random[2][0] = dir1[2];

      random[0][1] = dir2[0];
      random[1][1] = dir2[1];
      random[2][1] = dir2[2];

      random[0][2] = dir3[0];
      random[1][2] = dir3[1];
      random[2][2] = dir3[2];

      // compute volume according to temporary orientation
      point_t low, high;
      limits(point_begin, point_end, center, random, low, high);

      oriented_boundingbox<point_t> obb(random, center, low, high);
      value_type volume = obb.volume();

      // if tighter, apply
      if (volume < tightest_volume) {
        orientation = random;
        tightest_volume = volume;
      }
    }

    return orientation;
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

  //////////////////////////////////////////////////////////////////////////////
  point_t generate_random_point() const {
    //srand ( 7 );
    point_t random;
    for (unsigned i = 0; i != point_t::size; ++i) {
      random[i] =
          value_type(2) * (value_type(rand()) / RAND_MAX) - value_type(0.5);
    }
    return random;
  }

 private:

  std::size_t _iterations;

};

}  // namespace tml

#endif  // TML_ORIENTED_BOUNDINGBOX_RANDOM_POLICY_HPP
