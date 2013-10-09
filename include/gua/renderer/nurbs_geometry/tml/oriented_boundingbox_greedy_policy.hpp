/********************************************************************************
*
* Copyright (C) 2011 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : oriented_boundingbox_greedy_policy.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_ORIENTED_BOUNDINGBOX_GREEDY_POLICY_HPP
#define TML_ORIENTED_BOUNDINGBOX_GREEDY_POLICY_HPP

// header, system
#include <list>
#include <boost/array.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/matrix.hpp>

namespace tml {

template <typename point_t> class greedy_policy {
 public:  // typedefs

  static unsigned const N = point_t::coordinates;

  typedef typename point_t::value_type value_type;
  typedef point_t point_type;
  typedef matrix<value_type, N, N> matrix_type;

 public:

  //////////////////////////////////////////////////////////////////////////////
  greedy_policy(pointmesh2d<point_t> const& mesh,
                std::size_t iterations = 1000,
                value_type step = 0.01)
      : _iterations(iterations), _stepwidth(step) {}

  //////////////////////////////////////////////////////////////////////////////
  greedy_policy(pointmesh3d<point_t> const& mesh,
                std::size_t iterations = 1000,
                value_type step = 0.01)
      : _iterations(iterations), _stepwidth(step) {}

  //////////////////////////////////////////////////////////////////////////////
  greedy_policy(std::size_t iterations = 300, value_type step = 0.05)
      : _iterations(iterations), _stepwidth(step) {}

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
    matrix_type orientation;
    value_type tightest_volume = boost::numeric::bounds<value_type>::highest();

    for (unsigned int k = 0; k != _iterations; ++k) {
      // temporaries for dimension
      point_t low, high;

      // compute next candidate
      value_type angle =
          value_type(2.0) * (value_type(rand()) / RAND_MAX - value_type(0.5));

      matrix_type candidate;
      std::size_t dim = rand() % point_type::coordinates;

      switch (dim) {
        case 0: {
          candidate = orientation * make_rotation_x(angle * _stepwidth);
          break;
        }
        case 1: {
          candidate = orientation * make_rotation_y(angle * _stepwidth);
          break;
        }
        case 2: {
          candidate = orientation * make_rotation_z(angle * _stepwidth);
          break;
        }
      }

      // compute volume for new orientation
      limits(point_begin, point_end, center, candidate, low, high);
      oriented_boundingbox<point_t> obb(candidate, center, low, high);
      value_type volume = obb.volume();

      // if tighter, apply
      if (volume < tightest_volume) {
        orientation = candidate;
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

      for (unsigned i = 0; i != point_t::coordinates; ++i) {
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
    srand(8);
    point_t random;
    for (unsigned i = 0; i != point_t::size; ++i) {
      random[i] =
          value_type(2) * (value_type(rand()) / RAND_MAX) - value_type(1);
    }
    return random;
  }

 private:

  std::size_t _iterations;
  value_type _stepwidth;

};

}  // namespace tml

#endif  // TML_ORIENTED_BOUNDINGBOX_GREEDY_POLICY_HPP
