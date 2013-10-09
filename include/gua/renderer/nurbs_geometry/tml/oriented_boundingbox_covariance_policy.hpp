/********************************************************************************
*
* Copyright (C) 2011 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : oriented_boundingbox_covariance_policy.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_ORIENTED_BOUNDINGBOX_COVARIANCE_POLICY_HPP
#define TML_ORIENTED_BOUNDINGBOX_COVARIANCE_POLICY_HPP

// header, system
#include <list>
#include <boost/array.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/matrix.hpp>

namespace tml {

template <typename point_t> class covariance_policy {
 public:  // typedefs

  static unsigned const N = point_t::coordinates;

  typedef typename point_t::value_type value_type;
  typedef point_t point_type;
  typedef matrix<value_type, N, N> matrix_type;

 public:

  //////////////////////////////////////////////////////////////////////////////
  covariance_policy(pointmesh2d<point_t> const& mesh,
                    value_type const& rootfind_min = -1000.0,
                    value_type const& rootfind_max = 1000.0,
                    value_type const& rootfind_eps = 0.000001)
      : _rfmin(rootfind_min), _rfmax(rootfind_max), _rfeps(rootfind_eps) {}

  //////////////////////////////////////////////////////////////////////////////
  covariance_policy(pointmesh3d<point_t> const& mesh,
                    value_type const& rootfind_min = -1000.0,
                    value_type const& rootfind_max = 1000.0,
                    value_type const& rootfind_eps = 0.000001)
      : _rfmin(rootfind_min), _rfmax(rootfind_max), _rfeps(rootfind_eps) {}

  //////////////////////////////////////////////////////////////////////////////
  covariance_policy(value_type const& rootfind_min = -1000.0,
                    value_type const& rootfind_max = 1000.0,
                    value_type const& rootfind_eps = 0.000001)
      : _rfmin(rootfind_min), _rfmax(rootfind_max), _rfeps(rootfind_eps) {}

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
    matrix<value_type, N, N> C;

    for (std::size_t row = 0; row != N; ++row) {
      for (std::size_t col = 0; col != N; ++col) {
        for (iterator_t p = point_begin; p != point_end; ++p) {
          C[row][col] = ((*p) - center)[row] * ((*p) - center)[col];
        }
      }
    }

    C /= value_type(std::distance(point_begin, point_end));
    std::vector<point<value_type, point_t::coordinates> > eigenvectors =
        C.eigenvectors(_rfmin, _rfmax, _rfeps);

    matrix<value_type, N, N> orientation;

    /*if ( eigenvectors.size() == point_t::size )
      {
        for (unsigned c = 0; c != point_t::size; ++c) {
          for (unsigned r = 0; r != point_t::size; ++r) {
            orientation[c][r] = eigenvectors[c][r];
          }
        }

        return orientation;
      } else {*/
    if (!eigenvectors.empty()) {
      // handle case of one eigenvector
      switch (point_t::coordinates) {
        case 3: {
          // set second axis
          point_t b;
          b[0] = 0;
          b[1] = -eigenvectors[0][2];
          b[2] = eigenvectors[0][1];

          // normalize
          b.normalize();

          // compute third axis
          point_t c = cross(point_t(eigenvectors[0]), point_t(b));

          orientation[0][0] = eigenvectors[0][0];
          orientation[1][0] = eigenvectors[0][1];
          orientation[2][0] = eigenvectors[0][2];

          orientation[0][1] = b[0];
          orientation[1][1] = b[1];
          orientation[2][1] = b[2];

          orientation[0][2] = c[0];
          orientation[1][2] = c[1];
          orientation[2][2] = c[2];
          break;
        }

        case 2: {
          if (eigenvectors.size() == 2) {
            orientation[0][0] = eigenvectors[0][0];
            orientation[1][0] = eigenvectors[0][1];
            orientation[0][1] = eigenvectors[1][0];
            orientation[1][1] = eigenvectors[1][1];
          } else {
            throw std::runtime_error(
                "Not enough Eigenvalues to determine orientation.");
          }
          break;
        }

        default: {
          throw std::runtime_error("Couldn't compute OBB orientation.");
        }
      }
    } else {
      throw std::runtime_error("Couldn't compute OBB orientation.");
    }
    // }

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

 private:

  value_type _rfmin;
  value_type _rfmax;
  value_type _rfeps;
};

}  // namespace tml

#endif  // TML_ORIENTED_BOUNDINGBOX_COVARIANCE_POLICY_HPP
