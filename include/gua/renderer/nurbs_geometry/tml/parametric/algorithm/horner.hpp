/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : horner.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_HORNER_HPP
#define TML_HORNER_HPP

// header, system
#include <algorithm>
#include <functional>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/evaluator.hpp>

namespace tml {

// horner scheme for rational curves
template <typename point_t> class horner : public evaluator<point_t> {
 public:  //typedefs

  // typedef typename std::vector<point_t>::iterator point_iterator;
  typedef typename evaluator<point_t>::point_iterator point_iterator;
  typedef typename evaluator<point_t>::const_point_iterator
      const_point_iterator;
  typedef typename point_t::value_type value_type;

 public:  // c'tor

  horner() : evaluator<point_t>() {}

  /* virtual */ ~horner() {}

 public:  // method

  /////////////////////////////////////////////////////////////////////////////
  // Evaluating point on rational bezier curve
  // using horner scheme in bernstein basis
  /////////////////////////////////////////////////////////////////////////////
  /* virtual */ void operator()(/* pointer to first control point */
                                const_point_iterator points,
                                /* order of curve */
                                std::size_t order,
                                /* t-parameter to evaluate at */
                                value_type t, /* resulting point */
                                point_t& point) const {
    // helper like binomial coefficients and t^n
    value_type one_minus_t = value_type(1) - t;
    value_type bc = 1;
    value_type tn = 1;

    // first interpolation (1-t)^n
    point_t tmp = points->as_homogenous() * one_minus_t;

    // evaluate using horner scheme
    for (std::size_t i = 1; i < order - 1; ++i) {
      tn *= t;
      bc *= value_type(order - i) / value_type(i);

      ++points;  // go to next point
      tmp = (tmp + tn * bc * points->as_homogenous()) * one_minus_t;
    }

    // last interpolation t^n
    ++points;  // go to next point
    point = tmp + tn * t * points->as_homogenous();

    // project back into E3
    point = point.as_euclidian();
  }

  /////////////////////////////////////////////////////////////////////////////
  // Evaluating point on rational bezier curve
  // using horner scheme in bernstein basis
  /////////////////////////////////////////////////////////////////////////////
  /* virtual */ void operator()(/* pointer to first control point */
                                const_point_iterator points,
                                /* order of curve */
                                std::size_t order,
                                /* t-parameter to evaluate at */
                                value_type t,   /* resulting point */
                                point_t& point, /* resulting derivative */
                                point_t& dt) const {

    // helper like binomial coefficients and t^n
    value_type u = value_type(1.0) - t;
    value_type bc = 1;
    value_type tn = 1;

    // first interpolation (1-t)^n
    point_t tmp0 = (*points).as_homogenous() * u;
    point_t tmp1 = (*(points + 1)).as_homogenous() * u;

    // evaluate using horner scheme
    for (std::size_t i = 1; i < order - 2; ++i) {
      tn *= t;
      bc *= value_type(order - i - 1) / value_type(i);

      tmp0 = (tmp0 + tn * bc * (*(points + i)).as_homogenous()) * u;
      tmp1 = (tmp1 + tn * bc * (*(points + i + 1)).as_homogenous()) * u;
    }

    // last interpolation t^n
    tmp0 = tmp0 + tn * t * (*(points + order - 2)).as_homogenous();
    tmp1 = tmp1 + tn * t * (*(points + order - 1)).as_homogenous();

    // result in hyperspace
    point = u * tmp0 + t * tmp1;

    // compute derivative after value_typeer '91
    dt = value_type(order - 1) * (((tmp0.weight() * tmp1.weight()) /
                                   (point.weight() * point.weight()))) *
         (tmp1.as_euclidian() - tmp0.as_euclidian());

    // project back into E3
    point = point.as_euclidian();
  }

  /////////////////////////////////////////////////////////////////////////////
  // surface evaluation using horner scheme
  /////////////////////////////////////////////////////////////////////////////
  /* virtual */ point_t operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* u-parameter for point to evaluate */
      value_type u,                /* v-parameter for point to evaluate */
      value_type v) const {
    point_t tmp;
    (*this)(points, order_u, order_v, u, v, tmp);
    return tmp;
  }

  /* virtual */ void operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* u-parameter for point to evaluate */
      value_type u,                /* v-parameter for point to evaluate */
      value_type v,                /* resulting point at [u,v] */
      point_t& point) const {
    value_type bcu = 1.0;
    value_type un = 1.0;
    std::size_t deg_u = order_u - 1;
    std::size_t deg_v = order_v - 1;

    point_t u0_0 = (*(points)).as_homogenous() * (value_type(1.0) - u);
    point_t u0_1 = (*(points + 1)).as_homogenous() * (value_type(1.0) - u);
    point_t u1_0 =
        (*(points + order_u)).as_homogenous() * (value_type(1.0) - u);
    point_t u1_1 =
        (*(points + order_u + 1)).as_homogenous() * (value_type(1.0) - u);

    /**************************************** 1. step : horner for first 2 rows
     * *********************************/
    for (std::size_t i = 1; i <= deg_u - 2; ++i) {
      un = un * u;
      bcu = bcu * (value_type(deg_u - i) / value_type(i));

      u0_0 = (u0_0 + un * bcu * (*(points + i)).as_homogenous()) *
             (value_type(1.0) - u);
      u0_1 = (u0_1 + un * bcu * (*(points + i + 1)).as_homogenous()) *
             (value_type(1.0) - u);

      u1_0 = (u1_0 + un * bcu * (*(points + order_u + i)).as_homogenous()) *
             (value_type(1.0) - u);
      u1_1 = (u1_1 + un * bcu * (*(points + order_u + i + 1)).as_homogenous()) *
             (value_type(1.0) - u);
    }

    u0_0 += un * u * (*(points + deg_u - 1)).as_homogenous();
    u0_1 += un * u * (*(points + deg_u)).as_homogenous();
    u1_0 += un * u * (*(points + order_u + deg_u - 1)).as_homogenous();
    u1_1 += un * u * (*(points + order_u + deg_u)).as_homogenous();

    /* point in first and second row */
    point_t u0 = (value_type(1.0) - u) * u0_0 + u * u0_1;
    point_t u1 = (value_type(1.0) - u) * u1_0 + u * u1_1;

    /**************************************** 2. step : inner loop for rows 3 to
     * order - 1 ***********************/
    value_type bcv = 1.0;
    value_type vn = 1.0;

    point_t v0 = u0 * (value_type(1.0) - v);
    point_t v1 = u1 * (value_type(1.0) - v);

    point_t ui, ui_0, ui_1;
    for (std::size_t i = 1; i <= deg_v - 2; ++i) {
      bcu = 1.0;
      un = 1.0;
      ui_0 = (*(points + (i + 1) * order_u)).as_homogenous() *
             (value_type(1.0) - u);
      ui_1 = (*(points + (i + 1) * order_u + 1)).as_homogenous() *
             (value_type(1.0) - u);

      for (std::size_t j = 1; j <= deg_u - 2; ++j) {
        un = un * u;
        bcu = bcu * (value_type(deg_u - j) / value_type(j));
        ui_0 = (ui_0 + un * bcu * (*(points + (i + 1) * order_u + j))
                                      .as_homogenous()) * (value_type(1.0) - u);
        ui_1 = (ui_1 + un * bcu * (*(points + (i + 1) * order_u + j + 1))
                                      .as_homogenous()) * (value_type(1.0) - u);
      }
      ui_0 = ui_0 + un * u * (*(points + (i + 1) * order_u + deg_u - 1))
                                 .as_homogenous();
      ui_1 = ui_1 +
             un * u * (*(points + (i + 1) * order_u + deg_u)).as_homogenous();
      ui = (value_type(1.0) - u) * ui_0 + u * ui_1;

      u0 = u1;
      u1 = ui;

      vn = vn * v;
      bcv = bcv * (value_type(deg_v - i) / value_type(i));
      v0 = (v0 + vn * bcv * u0) * (value_type(1.0) - v);
      v1 = (v1 + vn * bcv * u1) * (value_type(1.0) - v);
    }

    /**************************************** 3. step : horner scheme for last
     * row *******************************/
    bcu = 1.0;
    un = 1.0;
    ui_0 =
        (*(points + deg_v * order_u)).as_homogenous() * (value_type(1.0) - u);
    ui_1 = (*(points + deg_v * order_u + 1)).as_homogenous() *
           (value_type(1.0) - u);

    for (std::size_t i = 1; i <= deg_u - 2; ++i) {
      un = un * u;
      bcu = bcu * (value_type(deg_u - i) / value_type(i));
      ui_0 = (ui_0 + un * bcu * (*(points + deg_v * order_u + i))
                                    .as_homogenous()) * (value_type(1.0) - u);
      ui_1 = (ui_1 + un * bcu * (*(points + deg_v * order_u + i + 1))
                                    .as_homogenous()) * (value_type(1.0) - u);
    }

    ui_0 = ui_0 +
           un * u * (*(points + deg_v * order_u + deg_u - 1)).as_homogenous();
    ui_1 =
        ui_1 + un * u * (*(points + deg_v * order_u + deg_u)).as_homogenous();
    ui = (value_type(1.0) - u) * ui_0 + u * ui_1;

    /**************************************** 4. step : final interpolation over
     * v ********************************/
    v0 += vn * v * u1;
    v1 += vn * v * ui;

    point = (value_type(1.0) - v) * v0 + v * v1;
    point = point.as_euclidian();
  }

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
    value_type bcu = 1.0;
    value_type un = 1.0;
    std::size_t deg_u = order_u - 1;
    std::size_t deg_v = order_v - 1;

    point_t u0_0 = (*(points)).as_homogenous() * (value_type(1.0) - u);
    point_t u0_1 = (*(points + 1)).as_homogenous() * (value_type(1.0) - u);
    point_t u1_0 =
        (*(points + order_u)).as_homogenous() * (value_type(1.0) - u);
    point_t u1_1 =
        (*(points + order_u + 1)).as_homogenous() * (value_type(1.0) - u);

    /**************************************** 1. step : horner for first 2 rows
     * *********************************/
    for (std::size_t i = 1; i <= deg_u - 2; ++i) {
      un = un * u;
      bcu = bcu * (value_type(deg_u - i) / value_type(i));

      u0_0 = (u0_0 + un * bcu * (*(points + i)).as_homogenous()) *
             (value_type(1.0) - u);
      u0_1 = (u0_1 + un * bcu * (*(points + i + 1)).as_homogenous()) *
             (value_type(1.0) - u);

      u1_0 = (u1_0 + un * bcu * (*(points + order_u + i)).as_homogenous()) *
             (value_type(1.0) - u);
      u1_1 = (u1_1 + un * bcu * (*(points + order_u + i + 1)).as_homogenous()) *
             (value_type(1.0) - u);
    }

    u0_0 += un * u * (*(points + deg_u - 1)).as_homogenous();
    u0_1 += un * u * (*(points + deg_u)).as_homogenous();
    u1_0 += un * u * (*(points + order_u + deg_u - 1)).as_homogenous();
    u1_1 += un * u * (*(points + order_u + deg_u)).as_homogenous();

    /* point in first and second row */
    point_t u0 = (value_type(1.0) - u) * u0_0 + u * u0_1;
    point_t u1 = (value_type(1.0) - u) * u1_0 + u * u1_1;

    /**************************************** 2. step : inner loop for rows 3 to
     * order - 1 ***********************/
    value_type bcv = 1.0;
    value_type vn = 1.0;

    point_t v0 = u0 * (value_type(1.0) - v);
    point_t v1 = u1 * (value_type(1.0) - v);

    point_t ui, ui_0, ui_1;
    for (std::size_t i = 1; i <= deg_v - 2; ++i) {
      bcu = 1.0;
      un = 1.0;
      ui_0 = (*(points + (i + 1) * order_u)).as_homogenous() *
             (value_type(1.0) - u);
      ui_1 = (*(points + (i + 1) * order_u + 1)).as_homogenous() *
             (value_type(1.0) - u);

      for (std::size_t j = 1; j <= deg_u - 2; ++j) {
        un = un * u;
        bcu = bcu * (value_type(deg_u - j) / value_type(j));
        ui_0 = (ui_0 + un * bcu * (*(points + (i + 1) * order_u + j))
                                      .as_homogenous()) * (value_type(1.0) - u);
        ui_1 = (ui_1 + un * bcu * (*(points + (i + 1) * order_u + j + 1))
                                      .as_homogenous()) * (value_type(1.0) - u);
      }
      ui_0 = ui_0 + un * u * (*(points + (i + 1) * order_u + deg_u - 1))
                                 .as_homogenous();
      ui_1 = ui_1 +
             un * u * (*(points + (i + 1) * order_u + deg_u)).as_homogenous();
      ui = (value_type(1.0) - u) * ui_0 + u * ui_1;

      u0 = u1;
      u1 = ui;

      vn = vn * v;
      bcv = bcv * (value_type(deg_v - i) / value_type(i));
      v0 = (v0 + vn * bcv * u0) * (value_type(1.0) - v);
      v1 = (v1 + vn * bcv * u1) * (value_type(1.0) - v);
    }

    /**************************************** 3. step : horner scheme for last
     * row *******************************/
    bcu = 1.0;
    un = 1.0;
    ui_0 =
        (*(points + deg_v * order_u)).as_homogenous() * (value_type(1.0) - u);
    ui_1 = (*(points + deg_v * order_u + 1)).as_homogenous() *
           (value_type(1.0) - u);

    for (std::size_t i = 1; i <= deg_u - 2; ++i) {
      un = un * u;
      bcu = bcu * (value_type(deg_u - i) / value_type(i));
      ui_0 = (ui_0 + un * bcu * (*(points + deg_v * order_u + i))
                                    .as_homogenous()) * (value_type(1.0) - u);
      ui_1 = (ui_1 + un * bcu * (*(points + deg_v * order_u + i + 1))
                                    .as_homogenous()) * (value_type(1.0) - u);
    }

    ui_0 = ui_0 +
           un * u * (*(points + deg_v * order_u + deg_u - 1)).as_homogenous();
    ui_1 =
        ui_1 + un * u * (*(points + deg_v * order_u + deg_u)).as_homogenous();
    ui = (value_type(1.0) - u) * ui_0 + u * ui_1;

    /**************************************** 4. step : final interpolation over
     * v ********************************/
    v0 += vn * v * u1;
    v1 += vn * v * ui;

    point = (value_type(1.0) - v) * v0 + v * v1;

    /* transform to euclidian space */
    dv = (order_v - 1) *
         ((v0.weight() * v1.weight()) / (point.weight() * point.weight())) *
         ((v1 / v1.weight()) - (v0 / v0.weight()));

    /************************************ 5.step : dartial derivative over u
     * ***********************************/
    bcv = 1.0;
    vn = 1.0;

    point_t v0_0 = (*(points)).as_homogenous() * (value_type(1.0) - v);
    point_t v0_1 =
        (*(points + order_u)).as_homogenous() * (value_type(1.0) - v);
    point_t v1_0 = (*(points + 1)).as_homogenous() * (value_type(1.0) - v);
    point_t v1_1 =
        (*(points + order_u + 1)).as_homogenous() * (value_type(1.0) - v);

    for (std::size_t i = 1; i <= deg_v - 2; ++i) {
      vn = vn * v;
      bcv = bcv * (value_type(deg_v - i) / value_type(i));

      v0_0 = (v0_0 + vn * bcv * (*(points + (i) * order_u)).as_homogenous()) *
             (value_type(1.0) - v);
      v0_1 =
          (v0_1 + vn * bcv * (*(points + (i + 1) * order_u)).as_homogenous()) *
          (value_type(1.0) - v);

      v1_0 =
          (v1_0 + vn * bcv * (*(points + (i) * order_u + 1)).as_homogenous()) *
          (value_type(1.0) - v);
      v1_1 = (v1_1 + vn * bcv * (*(points + (i + 1) * order_u + 1))
                                    .as_homogenous()) * (value_type(1.0) - v);
    }

    v0_0 = v0_0 + vn * v * (*(points + (deg_v - 1) * order_u)).as_homogenous();
    v0_1 = v0_1 + vn * v * (*(points + (deg_v) * order_u)).as_homogenous();
    v1_0 =
        v1_0 + vn * v * (*(points + (deg_v - 1) * order_u + 1)).as_homogenous();
    v1_1 = v1_1 + vn * v * (*(points + (deg_v) * order_u + 1)).as_homogenous();

    /* point in first and second row */
    v0 = (value_type(1.0) - v) * v0_0 + v * v0_1;
    v1 = (value_type(1.0) - v) * v1_0 + v * v1_1;

    /*********************************** 6. step : for all columns
     * *******************************************/
    bcu = 1.0;
    un = 1.0;

    u0 = v0 * (value_type(1.0) - u);
    u1 = v1 * (value_type(1.0) - u);

    point_t vi_0, vi_1, vi;
    for (std::size_t i = 1; i <= deg_u - 2; ++i) {
      bcv = 1.0;
      vn = 1.0;
      vi_0 = (*(points + i + 1)).as_homogenous() * (value_type(1.0) - v);
      vi_1 =
          (*(points + order_u + i + 1)).as_homogenous() * (value_type(1.0) - v);

      for (std::size_t j = 1; j <= deg_v - 2; ++j) {
        vn = vn * v;
        bcv = bcv * (value_type(deg_v - j) / value_type(j));
        vi_0 = (vi_0 + vn * bcv * (*(points + (j) * order_u + i + 1))
                                      .as_homogenous()) * (value_type(1.0) - v);
        vi_1 = (vi_1 + vn * bcv * (*(points + (j + 1) * order_u + i + 1))
                                      .as_homogenous()) * (value_type(1.0) - v);
      }
      vi_0 = vi_0 + vn * v * (*(points + (deg_v - 1) * order_u + i + 1))
                                 .as_homogenous();
      vi_1 = vi_1 +
             vn * v * (*(points + (deg_v) * order_u + i + 1)).as_homogenous();
      vi = (value_type(1.0) - v) * vi_0 + v * vi_1;

      v0 = v1;
      v1 = vi;

      un = un * u;
      bcu = bcu * (value_type(deg_u - i) / value_type(i));
      u0 = (u0 + un * bcu * v0) * (value_type(1.0) - u);
      u1 = (u1 + un * bcu * v1) * (value_type(1.0) - u);
    }

    /********************************* 7. horner step for last column
     * ****************************************/
    bcv = 1.0;
    vn = 1.0;
    vi_0 = (*(points + deg_u)).as_homogenous() * (value_type(1.0) - v);
    vi_1 =
        (*(points + order_u + deg_u)).as_homogenous() * (value_type(1.0) - v);

    for (std::size_t i = 1; i <= deg_v - 2; ++i) {
      vn = vn * v;
      bcv = bcv * (value_type(deg_v - i) / value_type(i));
      vi_0 = (vi_0 + vn * bcv * (*(points + (i) * order_u + deg_u))
                                    .as_homogenous()) * (value_type(1.0) - v);
      vi_1 = (vi_1 + vn * bcv * (*(points + (i + 1) * order_u + deg_u))
                                    .as_homogenous()) * (value_type(1.0) - v);
    }

    vi_0 = vi_0 +
           vn * v * (*(points + (deg_v - 1) * order_u + deg_u)).as_homogenous();
    vi_1 =
        vi_1 + vn * v * (*(points + (deg_v) * order_u + deg_u)).as_homogenous();
    vi = (value_type(1.0) - v) * vi_0 + v * vi_1;

    /******************************* 8. final interpolation
     * ***************************************************/
    u0 += un * u * v1;
    u1 += un * u * vi;

    /* transform to euclidian space */
    du = (order_u - 1) *
         ((u0.weight() * u1.weight()) / (point.weight() * point.weight())) *
         (u1.as_euclidian() - u0.as_euclidian());
    point = point.as_euclidian();
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
    // helper like binomial coefficients and t^n
    value_type one_minus_u = value_type(1.0) - u;
    value_type one_minus_v = value_type(1.0) - v;
    value_type one_minus_w = value_type(1.0) - w;

    value_type bcw = 1;
    value_type wn = 1;

    // evaluate using horner scheme
    for (std::size_t k = 0; k != order_w; ++k) {
      point_t pv;
      value_type bcv = 1;
      value_type vn = 1;

      for (std::size_t j = 0; j != order_v; ++j) {
        point_t pu;
        value_type bcu = 1;
        value_type un = 1;

        for (std::size_t i = 0; i != order_u; ++i) {
          if (i == 0) {  // first interpolation (1-u)^n
            pu = (points + j * order_u + k * order_u * order_v)
                     ->as_homogenous() * one_minus_u;
          } else {
            if (i == order_u - 1) {  // last interpolation u^n
              pu += un * u * (points + i + j * order_u + k * order_u * order_v)
                                 ->as_homogenous();
            } else {  // else follow regular horner scheme
              un *= u;
              bcu *= value_type(order_u - i) / value_type(i);
              pu = (pu + un * bcu *
                             (points + i + j * order_u + k * order_u * order_v)
                                 ->as_homogenous()) * one_minus_u;
            }
          }
        }

        if (j == 0) {  // first interpolation (1-v)^n
          pv = pu * one_minus_v;
        } else {
          if (j == order_v - 1) {
            pv += vn * v * pu;
          } else {
            vn *= v;
            bcv *= value_type(order_v - j) / value_type(j);
            pv = (pv + vn * bcv * pu) * one_minus_v;
          }
        }
      }

      if (k == 0) {  // first interpolation (1-w)^n
        point = pv * one_minus_w;
      } else {
        if (k == order_w - 1) {
          point += wn * w * pv;
        } else {
          wn *= w;
          bcw *= value_type(order_w - k) / value_type(k);
          point = (point + wn * bcw * pv) * one_minus_w;
        }
      }
    }

    point.project_to_euclidian();
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
      value_type u,   /* value_typeparameter for point to evaluate */
      value_type v,   /* w-parameter for point to evaluate */
      value_type w,   /* resulting point at [u,v,w] */
      point_t& point, /* first partial derivative in u at [u,v,w] */
      point_t& du,    /* first partial derivative in v at [u,v,w] */
      point_t& dv,    /* first partial derivative in w at [u,v,w] */
      point_t& dw) const {
    // helper like binomial coefficients and t^n
    value_type one_minus_u = value_type(1.0) - u;
    value_type one_minus_v = value_type(1.0) - v;
    value_type one_minus_w = value_type(1.0) - w;

    value_type bcw0 = 1;
    value_type wn0 = 1;
    value_type bcw1 = 1;
    value_type wn1 = 1;

    point_t p000;
    point_t p001;
    point_t p010;
    point_t p011;
    point_t p100;
    point_t p101;
    point_t p110;
    point_t p111;

    // evaluate using horner scheme
    for (std::size_t k = 0; k != order_w; ++k) {
      point_t pv;

      point_t pv00;
      point_t pv01;
      point_t pv10;
      point_t pv11;

      value_type bcv0 = 1;
      value_type bcv1 = 1;
      value_type vn0 = 1;
      value_type vn1 = 1;

      for (std::size_t j = 0; j != order_v; ++j) {
        point_t pu;  // temporaryly
        point_t pu0;
        point_t pu1;
        value_type bcu = 1;
        value_type un = 1;

        // first interpolation (1-u)^n
        pu0 = (points + j * order_u + k * order_u * order_v)->as_homogenous() *
              one_minus_u;
        pu1 = (points + 1 + j * order_u + k * order_u * order_v)
                  ->as_homogenous() * one_minus_u;

        for (std::size_t i = 1; i != order_u - 2; ++i) {
          // follow regular horner scheme
          un *= u;
          bcu *= value_type(order_u - 1 - i) / value_type(i);
          pu0 = (pu0 +
                 un * bcu * (points + i + j * order_u + k * order_u * order_v)
                                ->as_homogenous()) * one_minus_u;
          pu1 = (pu1 + un * bcu * (points + 1 + i + j * order_u +
                                   k * order_u * order_v)
                                      ->as_homogenous()) * one_minus_u;
        }

        // last interpolation u^n
        pu0 += un * u * (points + order_u - 2 + j * order_u +
                         k * order_u * order_v)->as_homogenous();
        pu1 += un * u * (points + order_u - 1 + j * order_u +
                         k * order_u * order_v)->as_homogenous();

        pu = (1 - u) * pu0 + u * pu1;

        if (j == 0) {  // first interpolation (1-v)^n
          pv00 = pu0 * one_minus_v;
          pv10 = pu1 * one_minus_v;
        }

        if (j == 1) {
          pv01 = pu0 * one_minus_v;
          pv11 = pu1 * one_minus_v;
        }

        if (j == 1 && order_v > 3) {
          vn0 *= v;
          bcv0 *= value_type(order_v - 1 - j) / value_type(j);
          pv00 = (pv00 + vn0 * bcv0 * pu0) * one_minus_v;
          pv10 = (pv10 + vn0 * bcv0 * pu1) * one_minus_v;
        }

        if (j == order_v - 2) {
          pv00 += vn0 * v * pu0;
          pv10 += vn0 * v * pu1;
        }

        if (j == order_v - 2 && order_v > 3) {
          vn1 *= v;
          bcv1 *= value_type(order_v - j) / value_type(j - 1);
          pv01 = (pv01 + vn1 * bcv1 * pu0) * one_minus_v;
          pv11 = (pv11 + vn1 * bcv1 * pu1) * one_minus_v;
        }

        if (j == order_v - 1) {
          pv01 += vn1 * v * pu0;
          pv11 += vn1 * v * pu1;
        }

        if (j > 1 && j < order_v - 2 && order_v > 3) {
          vn0 *= v;
          vn1 *= v;
          bcv0 *= value_type(order_v - 1 - j) / value_type(j);
          bcv1 *= value_type(order_v - j) / value_type(j - 1);

          pv00 = (pv00 + vn0 * bcv0 * pu0) * one_minus_v;
          pv10 = (pv10 + vn0 * bcv0 * pu1) * one_minus_v;
          pv01 = (pv01 + vn1 * bcv1 * pu0) * one_minus_v;
          pv11 = (pv11 + vn1 * bcv1 * pu1) * one_minus_v;
        }
      }

      if (k == 0) {  // first interpolation (1-w)^n
        p000 = pv00 * one_minus_w;
        p100 = pv10 * one_minus_w;
        p010 = pv01 * one_minus_w;
        p110 = pv11 * one_minus_w;
      }

      if (k == 1) {
        p001 = pv00 * one_minus_w;
        p101 = pv10 * one_minus_w;
        p011 = pv01 * one_minus_w;
        p111 = pv11 * one_minus_w;
      }

      if (k == 1 && order_w > 3) {
        wn0 *= w;
        bcw0 *= value_type(order_w - 1 - k) / value_type(k);
        p000 = (p000 + wn0 * bcw0 * pv00) * one_minus_w;
        p100 = (p100 + wn0 * bcw0 * pv10) * one_minus_w;
        p010 = (p010 + wn0 * bcw0 * pv01) * one_minus_w;
        p110 = (p110 + wn0 * bcw0 * pv11) * one_minus_w;
      }

      if (k == order_w - 2) {
        p000 += wn0 * w * pv00;
        p100 += wn0 * w * pv10;
        p010 += wn0 * w * pv01;
        p110 += wn0 * w * pv11;
      }

      if (k == order_w - 2 && order_w > 3) {
        wn1 *= w;
        bcw1 *= value_type(order_w - k) / value_type(k - 1);

        p001 = (p001 + wn1 * bcw1 * pv00) * one_minus_w;
        p101 = (p101 + wn1 * bcw1 * pv10) * one_minus_w;
        p011 = (p011 + wn1 * bcw1 * pv01) * one_minus_w;
        p111 = (p111 + wn1 * bcw1 * pv11) * one_minus_w;
      }

      if (k == order_w - 1) {
        p001 += wn1 * w * pv00;
        p101 += wn1 * w * pv10;
        p011 += wn1 * w * pv01;
        p111 += wn1 * w * pv11;
      }

      if (k > 1 && k < order_w - 2 && order_w > 3) {
        wn0 *= w;
        wn1 *= w;
        bcw0 *= value_type(order_w - 1 - k) / value_type(k);
        bcw1 *= value_type(order_w - k) / value_type(k - 1);

        p000 = (p000 + wn0 * bcw0 * pv00) * one_minus_w;
        p100 = (p100 + wn0 * bcw0 * pv10) * one_minus_w;
        p010 = (p010 + wn0 * bcw0 * pv01) * one_minus_w;
        p110 = (p110 + wn0 * bcw0 * pv11) * one_minus_w;

        p001 = (p001 + wn1 * bcw1 * pv00) * one_minus_w;
        p101 = (p101 + wn1 * bcw1 * pv10) * one_minus_w;
        p011 = (p011 + wn1 * bcw1 * pv01) * one_minus_w;
        p111 = (p111 + wn1 * bcw1 * pv11) * one_minus_w;
      }
    }

    // evaluate for u leaving a linear patch dependending on v,w
    point_t vw00 = (value_type(1) - u) * p000 + u * p100;
    point_t vw10 = (value_type(1) - u) * p010 + u * p110;
    point_t vw01 = (value_type(1) - u) * p001 + u * p101;
    point_t vw11 = (value_type(1) - u) * p011 + u * p111;

    // evaluate for v leaving a linear patch dependending on u,w
    point_t uw00 = (value_type(1) - v) * p000 + v * p010;
    point_t uw10 = (value_type(1) - v) * p100 + v * p110;
    point_t uw01 = (value_type(1) - v) * p001 + v * p011;
    point_t uw11 = (value_type(1) - v) * p101 + v * p111;

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
