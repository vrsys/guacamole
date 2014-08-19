/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : decasteljau.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_DECASTELJAU_HPP
#define TML_DECASTELJAU_HPP

// header, system
#include <cassert>
#include <algorithm>
#include <functional>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/evaluator.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/space_adapter.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh3d.hpp>

namespace tml {

// horner scheme for rational curves
template <typename point_t> class decasteljau : public evaluator<point_t> {
 public:  //typedefs

  // typedef typename std::vector<point_t>::iterator point_iterator;
  typedef typename evaluator<point_t>::point_iterator point_iterator;
  typedef typename evaluator<point_t>::const_point_iterator
      const_point_iterator;
  typedef typename point_t::value_type value_type;

 public:  // c'tor

  decasteljau() : evaluator<point_t>() {}

  /* virtual */ ~decasteljau() {}

 public:  // method

  /////////////////////////////////////////////////////////////////////////////
  // Evaluating point on rational bezier curve
  // using decasteljau scheme
  /////////////////////////////////////////////////////////////////////////////
  /* virtual */ void operator()(/* pointer to first control point */
                                const_point_iterator points,
                                /* order of curve */
                                std::size_t order,
                                /* t-parameter to evaluate at */
                                value_type t, /* resulting point */
                                point_t& point) const {
    assert(order > 0);

    // special case : end points
    if (t == 0) {
      point = *(points);
      return;
    }

    if (t == 1) {
      point = *(points + order - 1);
      return;
    }

    // if no special case -> do decasteljau evaluation
    std::vector<point_t> tmp(order);
    std::transform(points,
                   points + order,
                   tmp.begin(),
                   point_to_homogenous_space_adapter<point_t>());

    // evaluate using decasteljau scheme
    for (std::size_t i = 0; i != order - 1; ++i) {
      for (std::size_t j = 0; j != order - 1 - i; ++j) {
        tmp[j] = (value_type(1) - t) * tmp[j] + t * tmp[j + 1];
      }
    }

    // project back into E3
    point = tmp[0].as_euclidian();
  }

  /////////////////////////////////////////////////////////////////////////////
  // Evaluating point on rational bezier curve
  // using decasteljau scheme
  /////////////////////////////////////////////////////////////////////////////
  /* virtual */ void operator()(/* pointer to first control point */
                                const_point_iterator points,
                                /* order of curve */
                                std::size_t order,
                                /* t-parameter to evaluate at */
                                value_type t,   /* resulting point */
                                point_t& point, /* resulting derivative */
                                point_t& dt) const {
    assert(order > 1);

    // special case : end points
    if (t == 0) {
      point = *(points);
      dt = value_type(order) * (*(points + 1) - *points);
      return;
    }

    if (t == 1) {
      point = *(points + order - 1);
      dt = value_type(order) * (*(points) - *(points + 1));
      return;
    }

    std::vector<point_t> tmp(order);
    std::transform(points,
                   points + order,
                   tmp.begin(),
                   point_to_homogenous_space_adapter<point_t>());

    // evaluate using decasteljau scheme
    for (std::size_t i = 0; i != order - 1; ++i) {
      for (std::size_t j = 0; j != order - 1 - i; ++j) {
        tmp[j] = (value_type(1) - t) * tmp[j] + t * tmp[j + 1];
      }
    }

    // project back into E3
    point = tmp[0].as_euclidian();

    point_t aux_point = (tmp[0] - t * tmp[1]) / (value_type(1) - t);

    // M.S. Floater '91 :
    //
    //             w[0]{n-1}(t) * w[1]{n-1}(t)
    // P'(t) = n * --------------------------- * P[1]{n-1}(t) - P[0]{n-1}(t)
    //                     w[0]{n})^2
    //
    // 1. recalculate overwritten helping point P[0, n-1]
    // 2. project P[0, n-1] and P[1, n-1] into plane w=1
    // 3. use formula above to find out the correct length of P'(t)

    dt = (order - value_type(1)) * ((aux_point.weight() * tmp[1].weight()) /
                                    (tmp[0].weight() * tmp[0].weight())) *
         (tmp[1].as_euclidian() - aux_point.as_euclidian());
  }

  /////////////////////////////////////////////////////////////////////////////
  // surface evaluation using decasteljau scheme
  /////////////////////////////////////////////////////////////////////////////
  /* virtual */ point_t operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* u-parameter for point to evaluate */
      value_type u,                /* v-parameter for point to evaluate */
      value_type v) const {
    point_t point;
    (*this)(points, order_u, order_v, u, v, point);

    return point;
  }

  /////////////////////////////////////////////////////////////////////////////
  // surface evaluation using decasteljau scheme
  /////////////////////////////////////////////////////////////////////////////
  /* virtual */ void operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* u-parameter for point to evaluate */
      value_type u,                /* v-parameter for point to evaluate */
      value_type v,                /* resulting point at [u,v] */
      point_t& point) const {
    std::vector<point_t> hypercurve(order_v);

    for (std::size_t vi = 0; vi != order_v; ++vi) {
      const_point_iterator curve_iter = points;
      std::advance(curve_iter, vi * order_u);
      (*this)(curve_iter, order_u, u, hypercurve[vi]);
    }

    (*this)(hypercurve.begin(), order_v, v, point);
  }

  /////////////////////////////////////////////////////////////////////////////
  // surface evaluation using decasteljau scheme
  /////////////////////////////////////////////////////////////////////////////
  /* virtual */ void operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* u-parameter for point to evaluate */
      value_type u,                /* v-parameter for point to evaluate */
      value_type v,                /* resulting point at [u,v] */
      point_t& point, /* resulting partial derivative according to u */
      point_t& du,    /* resulting partial derivative according to v */
      point_t& dv) const {
    std::vector<point_t> hypercurve_v(order_v);
    std::vector<point_t> hypercurve_u(order_u);

    for (std::size_t vi = 0; vi != order_v; ++vi) {
      const_point_iterator curve_iter = points;
      std::advance(curve_iter, vi * order_u);
      (*this)(curve_iter, order_u, u, hypercurve_v[vi]);
    }

    for (std::size_t ui = 0; ui != order_u; ++ui) {
      std::vector<point_t> tmp(order_v);
      const_point_iterator curve_iter = points;
      std::advance(curve_iter, ui);

      for (std::size_t vi = 0; vi != order_v; ++vi) {
        tmp[vi] = *curve_iter;

        if (vi != order_v - 1) {
          std::advance(curve_iter, order_u);
        }
      }

      (*this)(tmp.begin(), order_v, v, hypercurve_u[ui]);
    }

    (*this)(hypercurve_v.begin(), order_v, v, point, dv);
    (*this)(hypercurve_u.begin(), order_u, u, point, du);
  }

  // volume evaluation
  /* virtual */ void operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* order in w dir */
      std::size_t order_w,         /* u-parameter for point to evaluate */
      value_type u,                /* v-parameter for point to evaluate */
      value_type v,                /* w-parameter for point to evaluate */
      value_type w,                /* resulting point at [u,v,w] */
      point_t& point) const {
    pointmesh3d<point_t> mesh(points,
                              points + order_u * order_v * order_w,
                              order_u,
                              order_v,
                              order_w);

    // transform control points to homogenous space
    std::for_each(
        mesh.begin(), mesh.end(), std::mem_fn(&point_t::project_to_homogenous));

    // first decasteljau in u direction
    for (std::size_t jv = 0; jv != order_v; ++jv) {
      for (std::size_t jw = 0; jw != order_w; ++jw) {
        for (std::size_t i = 0; i != order_u - 1; ++i) {
          for (std::size_t j = 0; j != order_u - 1 - i; ++j) {
            mesh(j, jv, jw) =
                (value_type(1) - u) * mesh(j, jv, jw) + u * mesh(j + 1, jv, jw);
          }
        }
      }
    }

    // secondly decasteljau in v direction
    for (std::size_t ju = 0; ju != order_u; ++ju) {
      for (std::size_t jw = 0; jw != order_w; ++jw) {
        for (std::size_t i = 0; i != order_v - 1; ++i) {
          for (std::size_t j = 0; j != order_v - 1 - i; ++j) {
            mesh(ju, j, jw) =
                (value_type(1) - v) * mesh(ju, j, jw) + v * mesh(ju, j + 1, jw);
          }
        }
      }
    }

    // thirdly decasteljau until mesh(0,0,0) equlas the evaluated point
    for (std::size_t ju = 0; ju != order_u; ++ju) {
      for (std::size_t jv = 0; jv != order_v; ++jv) {
        for (std::size_t i = 0; i != order_w - 1; ++i) {
          for (std::size_t j = 0; j != order_w - 1 - i; ++j) {
            mesh(ju, jv, j) =
                (value_type(1) - w) * mesh(ju, jv, j) + w * mesh(ju, jv, j + 1);
          }
        }
      }
    }

    point = mesh(0, 0, 0).as_euclidian();
  }

  // volume evaluation
  /* virtual */ point_t operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* order in w dir */
      std::size_t order_w,         /* u-parameter for point to evaluate */
      value_type u,                /* v-parameter for point to evaluate */
      value_type v,                /* w-parameter for point to evaluate */
      value_type w) const {
    point_t result;
    (*this)(points, order_u, order_v, order_w, u, v, w, result);
    return result;
  }

  // volume evaluation with partial derivatives
  /* virtual */ void operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* order in w dir */
      std::size_t order_w,         /* u-parameter for point to evaluate */
      value_type u,                /* v-parameter for point to evaluate */
      value_type v,                /* w-parameter for point to evaluate */
      value_type w,                /* resulting point at [u,v,w] */
      point_t& point, /* first partial derivative in u at [u,v,w] */
      point_t& du,    /* first partial derivative in v at [u,v,w] */
      point_t& dv,    /* first partial derivative in w at [u,v,w] */
      point_t& dw) const {
    pointmesh3d<point_t> mesh(points,
                              points + order_u * order_v * order_w,
                              order_u,
                              order_v,
                              order_w);

    // transform control points to homogenous space
    std::for_each(
        mesh.begin(), mesh.end(), std::mem_fn(&point_t::project_to_homogenous));

    // first decasteljau in u direction until only u-linear volume is left for u
    for (std::size_t jv = 0; jv != order_v; ++jv) {
      for (std::size_t jw = 0; jw != order_w; ++jw) {
        for (std::size_t i = 0; i != order_u - 2; ++i) {
          for (std::size_t j = 0; j != order_u - 1 - i; ++j) {
            mesh(j, jv, jw) =
                (value_type(1) - u) * mesh(j, jv, jw) + u * mesh(j + 1, jv, jw);
          }
        }
      }
    }

    // secondly decasteljau in v direction until only uv-linear volume is left
    for (std::size_t ju = 0; ju != 2; ++ju) {
      for (std::size_t jw = 0; jw != order_w; ++jw) {
        for (std::size_t i = 0; i != order_v - 2; ++i) {
          for (std::size_t j = 0; j != order_v - 1 - i; ++j) {
            mesh(ju, j, jw) =
                (value_type(1) - v) * mesh(ju, j, jw) + v * mesh(ju, j + 1, jw);
          }
        }
      }
    }

    // thirdly decasteljau until only trilinear volume is left
    for (std::size_t ju = 0; ju != 2; ++ju) {
      for (std::size_t jv = 0; jv != 2; ++jv) {
        for (std::size_t i = 0; i != order_w - 2; ++i) {
          for (std::size_t j = 0; j != order_w - 1 - i; ++j) {
            mesh(ju, jv, j) =
                (value_type(1) - w) * mesh(ju, jv, j) + w * mesh(ju, jv, j + 1);
          }
        }
      }
    }

    // evaluate for u leaving a linear patch dependending on v,w
    point_t vw00 = (value_type(1) - u) * mesh(0, 0, 0) + u * mesh(1, 0, 0);
    point_t vw10 = (value_type(1) - u) * mesh(0, 1, 0) + u * mesh(1, 1, 0);
    point_t vw01 = (value_type(1) - u) * mesh(0, 0, 1) + u * mesh(1, 0, 1);
    point_t vw11 = (value_type(1) - u) * mesh(0, 1, 1) + u * mesh(1, 1, 1);

    // evaluate for v leaving a linear patch dependending on u,w
    point_t uw00 = (value_type(1) - v) * mesh(0, 0, 0) + v * mesh(0, 1, 0);
    point_t uw10 = (value_type(1) - v) * mesh(1, 0, 0) + v * mesh(1, 1, 0);
    point_t uw01 = (value_type(1) - v) * mesh(0, 0, 1) + v * mesh(0, 1, 1);
    point_t uw11 = (value_type(1) - v) * mesh(1, 0, 1) + v * mesh(1, 1, 1);

    // evaluating v,w plane for v resulting in last linear interpolation in w ->
    // to compute first partial derivative in w
    point_t w0 = (value_type(1) - v) * vw00 + v * vw10;
    point_t w1 = (value_type(1) - v) * vw01 + v * vw11;

    // evaluating v,w plane for w resulting in last linear interpolation in v ->
    // to compute first partial derivative in v
    point_t v0 = (value_type(1) - w) * vw00 + w * vw01;
    point_t v1 = (value_type(1) - w) * vw10 + w * vw11;

    // evaluating v,w plane for w resulting in last linear interpolation in v ->
    // to compute first partial derivative in v
    point_t u0 = (value_type(1) - w) * uw00 + w * uw01;
    point_t u1 = (value_type(1) - w) * uw10 + w * uw11;

    // last interpolation and back projection to euclidian space
    point = (value_type(1) - w) * w0 + w * w1;
    point.project_to_euclidian();

    // M.S. Floater '91 :
    //
    //             w[0]{n-1}(t) * w[1]{n-1}(t)
    // P'(t) = n * --------------------------- * P[1]{n-1}(t) - P[0]{n-1}(t)
    //                     w[0]{n})^2
    //
    // 1. recalculate overwritten helping point P[0, n-1]
    // 2. project P[0, n-1] and P[1, n-1] into plane w=1
    // 3. use formula above to find out the correct length of P'(t)

    du = (order_u - value_type(1)) *
         ((u0.weight() * u1.weight()) / (point.weight() * point.weight())) *
         (u1.as_euclidian() - u0.as_euclidian());
    dv = (order_v - value_type(1)) *
         ((v0.weight() * v1.weight()) / (point.weight() * point.weight())) *
         (v1.as_euclidian() - v0.as_euclidian());
    dw = (order_w - value_type(1)) *
         ((w0.weight() * w1.weight()) / (point.weight() * point.weight())) *
         (w1.as_euclidian() - w0.as_euclidian());
  }

};

}  // namespace tml

#endif  // TML_HORNER_HPP
