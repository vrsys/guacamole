/********************************************************************************
*
* Copyright (C) 2011 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : oriented_boundingbox_partial_derivative_policy.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_ORIENTED_BOUNDINGBOX_PARTIAL_DERIVATIVE_POLICY_HPP
#define TML_ORIENTED_BOUNDINGBOX_PARTIAL_DERIVATIVE_POLICY_HPP

// header, system
#include <list>
#include <boost/array.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh2d.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh3d.hpp>
#include <gua/renderer/nurbs_geometry/tml/matrix.hpp>

namespace tml {

template <typename point_t> class partial_derivative_policy {
 public:  // typedefs

  static unsigned const N = point_t::coordinates;

  typedef typename point_t::value_type value_type;
  typedef point_t point_type;
  typedef matrix<value_type, N, N> matrix_type;

 public:

  //////////////////////////////////////////////////////////////////////////////
  partial_derivative_policy(bool ortho = true)
      : _mesh_dimensions(0), _mesh_size(), _orthogonalize(ortho) {}

  //////////////////////////////////////////////////////////////////////////////
  partial_derivative_policy(pointmesh2d<point_t> const& mesh, bool ortho = true)
      : _mesh_dimensions(2), _mesh_size(), _orthogonalize(ortho) {
    _mesh_size[0] = mesh.width();
    _mesh_size[1] = mesh.height();
  }

  //////////////////////////////////////////////////////////////////////////////
  partial_derivative_policy(pointmesh3d<point_t> const& mesh, bool ortho = true)
      : _mesh_dimensions(3), _mesh_size(), _orthogonalize(ortho) {
    _mesh_size[0] = mesh.width();
    _mesh_size[1] = mesh.height();
    _mesh_size[2] = mesh.depth();
  }

  //////////////////////////////////////////////////////////////////////////////
  void apply(pointmesh2d<point_t> const& mesh) {
    _mesh_dimensions = 2;
    _mesh_size[0] = mesh.width();
    _mesh_size[1] = mesh.height();
  }

  //////////////////////////////////////////////////////////////////////////////
  void apply(pointmesh3d<point_t> const& mesh) {
    _mesh_dimensions = 3;
    _mesh_size[0] = mesh.width();
    _mesh_size[1] = mesh.height();
    _mesh_size[2] = mesh.depth();
  }

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

    if (_orthogonalize) {
      orthogonalize(begin, end, obb_center, obb_orientation, obb_low, obb_high);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  template <typename iterator_t>
  point_t mean(iterator_t point_begin, iterator_t point_end) const {
    point_t zero;
    zero.weight(0);
    point_t sum = std::accumulate(point_begin, point_end, zero);
    return sum / value_type(std::distance(point_begin, point_end));
  }

  //////////////////////////////////////////////////////////////////////////////
  template <typename iterator_t>
  matrix_type orientation(iterator_t point_begin,
                          iterator_t point_end,
                          point_t const& center) const {
    matrix_type M;

    switch (_mesh_dimensions) {
      case 2:  // mesh is two-dimensional grid
               {
        // C----------D
        // |    |     |
        // |----------|v
        // |    |     |
        // A -------- B
        //      u

        assert(_mesh_size[0] > 0 && _mesh_size[1] > 0);

        iterator_t A = point_begin;
        iterator_t B = point_begin + _mesh_size[0] - 1;
        iterator_t C = point_begin + _mesh_size[0] * (_mesh_size[1] - 1);
        iterator_t D = point_begin + _mesh_size[0] * _mesh_size[1] - 1;

        // orientation is average of edges

        switch (N) {
          case 2: {
            point_type dx = ((*B) - (*A)) + ((*D) - (*C));
            point_type dy = ((*C) - (*A)) + ((*D) - (*B));

            // normalize
            dx.normalize();
            dy.normalize();

            M[0][0] = dx[0];
            M[1][0] = dx[1];

            M[0][1] = dy[0];
            M[1][1] = dy[1];

            break;
          }
          case 3: {
            point_type du = ((*B) - (*A)) + ((*D) - (*C));
            point_type dv = ((*C) - (*A)) + ((*D) - (*B));
            point_type dw = cross(du, dv);

            // normalize
            du.normalize();
            dv.normalize();
            dw.normalize();

            M[0][0] = du[0];
            M[1][0] = du[1];
            M[2][0] = du[2];

            M[0][1] = dv[0];
            M[1][1] = dv[1];
            M[2][1] = dv[2];

            M[0][2] = dw[0];
            M[1][2] = dw[1];
            M[2][2] = dw[2];

            break;
          }
          default:
            throw std::runtime_error("not implemented yet");
            break;
        }
        break;
      }
      case 3: {
        assert(_mesh_size[0] > 0 && _mesh_size[1] > 0 && _mesh_size[2] > 0);

        iterator_t A000 = point_begin;
        iterator_t A100 = point_begin + (_mesh_size[0] - 1);
        iterator_t A101 = point_begin + (_mesh_size[0] - 1) +
                          _mesh_size[0] * _mesh_size[1] * (_mesh_size[2] - 1);
        iterator_t A110 = point_begin + (_mesh_size[0] - 1) +
                          _mesh_size[0] * (_mesh_size[1] - 1);
        iterator_t A111 = point_begin + (_mesh_size[0] - 1) +
                          _mesh_size[0] * (_mesh_size[1] - 1) +
                          _mesh_size[0] * _mesh_size[1] * (_mesh_size[2] - 1);
        iterator_t A011 = point_begin + +_mesh_size[0] * (_mesh_size[1] - 1) +
                          _mesh_size[0] * _mesh_size[1] * (_mesh_size[2] - 1);
        iterator_t A010 = point_begin + +_mesh_size[0] * (_mesh_size[1] - 1);
        iterator_t A001 =
            point_begin + +_mesh_size[0] * _mesh_size[1] * (_mesh_size[2] - 1);

#if 1
        // orientation is average of outer edges
        point_type du = ((*A100) - (*A000)) + ((*A101) - (*A001)) +
                        ((*A110) - (*A010)) + ((*A111) - (*A011));
        point_type dv = ((*A010) - (*A000)) + ((*A110) - (*A100)) +
                        ((*A011) - (*A001)) + ((*A111) - (*A101));
        point_type dw = ((*A001) - (*A000)) + ((*A011) - (*A010)) +
                        ((*A101) - (*A100)) + ((*A111) - (*A110));
#else
        // orienatation is average of all partial derivatives
        point_type du, dv, dw;

        for (unsigned w = 0; w != _mesh_size[point_t::w]; ++w) {
          for (unsigned v = 0; v != _mesh_size[point_t::v]; ++v) {
            for (unsigned u = 0; u != _mesh_size[point_t::u]; ++u) {
              if (u != _mesh_size[point_t::u] - 1) {
                iterator_t du0 = point_begin + u + _mesh_size[0] * v +
                                 _mesh_size[0] * _mesh_size[1] * w;
                iterator_t du1 = point_begin + u + 1 + _mesh_size[0] * v +
                                 _mesh_size[0] * _mesh_size[1] * w;
                du += (*du1) - (*du0);
              }

              if (v != _mesh_size[point_t::v] - 1) {
                iterator_t dv0 = point_begin + u + _mesh_size[0] * v +
                                 _mesh_size[0] * _mesh_size[1] * w;
                iterator_t dv1 = point_begin + u + _mesh_size[0] * (v + 1) +
                                 _mesh_size[0] * _mesh_size[1] * w;
                dv += (*dv1) - (*dv0);
              }

              if (w != _mesh_size[point_t::w] - 1) {
                iterator_t dw0 = point_begin + u + _mesh_size[0] * v +
                                 _mesh_size[0] * _mesh_size[1] * w;
                iterator_t dw1 = point_begin + u + _mesh_size[0] * v +
                                 _mesh_size[0] * _mesh_size[1] * (w + 1);
                dw += (*dw1) - (*dw0);
              }
            }
          }
        }

#endif

        // normalize
        du.normalize();
        dv.normalize();

        if (_mesh_size[2] == 1) {
          dw = cross(du, dv);
        }
        dw.normalize();

        M[0][0] = du[0];
        M[1][0] = du[1];
        M[2][0] = du[2];

        M[0][1] = dv[0];
        M[1][1] = dv[1];
        M[2][1] = dv[2];

        M[0][2] = dw[0];
        M[1][2] = dw[1];
        M[2][2] = dw[2];

        break;
      }
      default: {
        throw std::runtime_error("not implemented yet");
        break;
      }
    }
    ;

    return M;
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
    point_t size = obb_high - obb_low;

    point_t delta = obb_orientation * (mean);

    obb_low = size / value_type(-2);
    obb_high = size / value_type(2);

    obb_center += delta;
  }

  //////////////////////////////////////////////////////////////////////////////
  template <typename iterator_t>
  void orthogonalize(iterator_t begin,
                     iterator_t end,
                     point_t& obb_center,
                     matrix_type& obb_orientation,
                     point_t& obb_low,
                     point_t& obb_high) {

    switch (_mesh_dimensions) {
      case 2: {
        point_type du;
        (obb_orientation[0][0], obb_orientation[1][0]);
        point_type dv;
        (obb_orientation[0][1], obb_orientation[1][1]);

        point_type size = obb_high - obb_low;
        if (size[0] > size[1])  // keep longest
            {
          dv[0] = du[1];
          dv[1] = -du[0];
        } else {
          du[0] = dv[1];
          du[1] = -dv[0];
        }
        break;
      }
      case 3: {
        point_type du, dv, dw;

        du[0] = obb_orientation[0][0];
        du[1] = obb_orientation[1][0];
        du[2] = obb_orientation[2][0];

        dv[0] = obb_orientation[0][1];
        dv[1] = obb_orientation[1][1];
        dv[2] = obb_orientation[2][1];

        dw[0] = obb_orientation[0][2];
        dw[1] = obb_orientation[1][2];
        dw[2] = obb_orientation[2][2];

        point_type size = obb_high - obb_low;
        if (size[0] > size[1] && size[1] > size[2]) {  // u longest, w shortest
          dw = cross(du, dv);
          dv = cross(du, dw);
        }

        if (size[0] > size[2] && size[2] > size[1]) {  //u_longest_v_shortest
          dv = cross(du, dw);
          dw = cross(du, dv);
        }

        if (size[1] > size[2] && size[2] > size[0]) {  // v_longest_u_shortest
          du = cross(dv, dw);
          dw = cross(dv, du);
        }

        if (size[1] > size[0] && size[0] > size[2]) {  // v_longest_w_shortest
          dw = cross(dv, du);
          du = cross(dv, dw);
        }

        if (size[2] > size[1] && size[1] > size[0]) {  // w_longest_u_shortest
          du = cross(dw, dv);
          dv = cross(dw, du);
        }

        if (size[2] > size[0] && size[0] > size[1]) {  // w_longest_v_shortest
          dv = cross(dw, du);
          du = cross(dw, dv);
        }

        du.normalize();
        dv.normalize();
        dw.normalize();

        // map orientation to parameter axis
        iterator_t A000 = begin;
        iterator_t A100 = begin + (_mesh_size[0] - 1);
        iterator_t A010 = begin + _mesh_size[0] * (_mesh_size[1] - 1);
        iterator_t A001 =
            begin + _mesh_size[0] * _mesh_size[1] * (_mesh_size[2] - 1);

        point_type pu = (*A100) - (*A000);
        point_type pv = (*A010) - (*A000);
        point_type pw = (*A001) - (*A000);

        pu.normalize();
        pv.normalize();
        pw.normalize();

        // match direction
        value_type dot_uu = dot(pu, du);
        //value_type dot_uv = dot(pu, dv);
        //value_type dot_uw = dot(pu, dw);

        //value_type dot_vu = dot(pv, du);
        value_type dot_vv = dot(pv, dv);
        //value_type dot_vw = dot(pv, dw);

        //value_type dot_wu = dot(pw, du);
        //value_type dot_wv = dot(pw, dv);
        value_type dot_ww = dot(pw, dw);

        if (dot_uu < 0) {
          du *= -1.0;
        }

        if (dot_vv < 0) {
          dv *= -1.0;
        }

        if (dot_ww < 0) {
          dw *= -1.0;
        }

        obb_orientation[0][0] = du[0];
        obb_orientation[1][0] = du[1];
        obb_orientation[2][0] = du[2];

        obb_orientation[0][1] = dv[0];
        obb_orientation[1][1] = dv[1];
        obb_orientation[2][1] = dv[2];

        obb_orientation[0][2] = dw[0];
        obb_orientation[1][2] = dw[1];
        obb_orientation[2][2] = dw[2];

        break;
      }
      default:
        throw std::runtime_error("Not implemented");
    }

    // recompute limits
    limits(begin, end, obb_center, obb_orientation, obb_low, obb_high);

  }

 private:  // attributes

  std::size_t _mesh_dimensions;
  boost::array<std::size_t, 3>
      _mesh_size;  // use same data structure for pointmesh2d and pointmesh3d
  bool _orthogonalize;  // force orthogonal base vectors

};

}  // namespace tml

#endif  // TML_ORIENTED_BOUNDINGBOX_PARTIAL_DERIVATIVE_POLICY_HPP
