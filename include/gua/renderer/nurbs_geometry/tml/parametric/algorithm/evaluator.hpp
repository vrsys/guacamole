/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : evaluator.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_EVALUATOR_HPP
#define TML_EVALUATOR_HPP

// header, system
#include <vector>

// header, project

namespace tml {

template <typename point_t> class evaluator {
 public:  // typedefs

  typedef typename std::vector<point_t>::iterator point_iterator;
  typedef typename std::vector<point_t>::const_iterator const_point_iterator;
  typedef typename point_t::value_type value_t;

 public:  // c'tor

  evaluator() {}

  virtual ~evaluator() {}

 public:  // method

  // curve evaluation
  virtual void operator()(const_point_iterator point_iterator,
                          std::size_t order,
                          value_t t,
                          point_t& point) const = 0;

  // curve evaluation
  virtual void operator()(const_point_iterator point_iterator,
                          std::size_t order,
                          value_t t,
                          point_t& point,
                          point_t& dt) const = 0;

  // surface evaluation
  virtual point_t operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* u-parameter for point to evaluate */
      value_t u,                   /* v-parameter for point to evaluate */
      value_t v) const = 0;

  // surface evaluation
  virtual void operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* u-parameter for point to evaluate */
      value_t u,                   /* v-parameter for point to evaluate */
      value_t v,                   /* resulting point at [u,v] */
      point_t& point) const = 0;

  // surface evaluation
  virtual void operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* u-parameter for point to evaluate */
      value_t u,                   /* v-parameter for point to evaluate */
      value_t v,                   /* resulting point at [u,v] */
      point_t& point, /* resulting partial derivative according to u */
      point_t& du,    /* resulting partial derivative according to v */
      point_t& dv) const = 0;

  // volume evaluation
  virtual void operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* order in w dir */
      std::size_t order_w,         /* u-parameter for point to evaluate */
      value_t u,                   /* v-parameter for point to evaluate */
      value_t v,                   /* w-parameter for point to evaluate */
      value_t w,                   /* resulting point at [u,v,w] */
      point_t& point) const = 0;

  // volume evaluation
  virtual point_t operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* order in w dir */
      std::size_t order_w,         /* u-parameter for point to evaluate */
      value_t u,                   /* v-parameter for point to evaluate */
      value_t v,                   /* w-parameter for point to evaluate */
      value_t w) const = 0;

  // volume evaluation with partial derivatives
  virtual void operator()(
      /* pointer to first point in rowwise control point grid */
      const_point_iterator points, /* order in u dir */
      std::size_t order_u,         /* order in v dir */
      std::size_t order_v,         /* order in w dir */
      std::size_t order_w,         /* u-parameter for point to evaluate */
      value_t u,                   /* v-parameter for point to evaluate */
      value_t v,                   /* w-parameter for point to evaluate */
      value_t w,                   /* resulting point at [u,v,w] */
      point_t& point, /* first partial derivative in u at [u,v,w] */
      point_t& du,    /* first partial derivative in v at [u,v,w] */
      point_t& dv,    /* first partial derivative in w at [u,v,w] */
      point_t& dw) const = 0;

};

}  // namespace tml

#endif  // TML_EVALUATOR_HPP
