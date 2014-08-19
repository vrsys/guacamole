/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : beziercurve.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_BEZIERCURVE_HPP
#define TML_BEZIERCURVE_HPP

// header, system
#include <set>     // std::set
#include <vector>  // std::vector

// header, project
#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox.hpp>
#include <gua/renderer/nurbs_geometry/tml/interval.hpp>
#include <gua/renderer/nurbs_geometry/tml/polynomial.hpp>

#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/evaluator.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/decasteljau.hpp>

namespace tml {

////////////////////////////////////////////////////////////////////////////////
// beziercurve - including control polygon and degree //////////////////////////
////////////////////////////////////////////////////////////////////////////////
template <typename point_t> class beziercurve {
 public:  // typedefs

  static unsigned const MAX_BINARY_SPLITS = 64;

  typedef point_t point_type;
  typedef typename point_t::value_type value_type;
  typedef typename std::vector<point_t>::iterator point_iterator;
  typedef typename std::vector<point_t>::const_iterator const_point_iterator;
  typedef beziercurve<point<value_type, 2> > beziercurve2_type;

 public:  // c'tors and d'tor

  beziercurve();
  beziercurve(beziercurve const&);
  beziercurve(std::size_t order);

  template <typename iterator_t> beziercurve(iterator_t beg, iterator_t end);

  template <typename container_t> beziercurve(container_t const& c);

  virtual ~beziercurve();

 public:  // methods

  virtual void swap(beziercurve& swp);
  virtual beziercurve& operator=(beziercurve const& cpy);

  // add point to curve manually
  void add(point_t const& point);

  template <typename external_point_type>
  void add(external_point_type const& point);

  // evaluating curve using arbitrary evaluator
  point_t evaluate(value_type const& t,
                   evaluator<point_t> const& evaluator =
                       decasteljau<point_t>()) const;

  void evaluate(value_type const& t,
                point_t& point,
                evaluator<point_t> const& evaluator =
                    decasteljau<point_t>()) const;

  void evaluate(value_type const& t,
                point_t& point,
                point_t& dt,
                evaluator<point_t> const& evaluator =
                    decasteljau<point_t>()) const;

  // degree and order
  std::size_t degree() const;
  std::size_t order() const;

  // print out
  virtual void print(std::ostream& os, std::string const& add_info = "") const;

  // derivative representation
  value_type curvature() const;  // 0 (flat) - 1 (curved)
  beziercurve scaled_hodograph() const;
  beziercurve hodograph() const;

  // transforms values on axis dim to equidistant 2d control points
  beziercurve2_type nishita(std::size_t dim) const;
  polynomial<value_type> as_polynomial(std::size_t coord) const;

  // splitting methods
  void clip_left(value_type t);
  void clip_right(value_type t);

  void clip_lr(value_type l, value_type r);

  void split(value_type t, beziercurve& left, beziercurve& right) const;

  template <typename parameter_container_t, typename curve_container_t>
  void split(parameter_container_t const& ordered_split_parameter,
             curve_container_t& curvesegments) const;

  void extrema(std::size_t dim,
               std::set<value_type>& roots,
               std::size_t iterations = MAX_BINARY_SPLITS) const;

  bool weak_monotonic(std::size_t dim) const;

  bool assert_extremum(std::size_t dim,
                       value_type const& t,
                       value_type const& dt) const;

  void bisect(std::size_t ray_direction,
              value_type const& direction_value,
              bool& isroot,
              value_type& root_parameter,
              interval<value_type> const& roots_parameter_range =
                  interval<value_type>(0, 1),
              std::size_t max_iterations = MAX_BINARY_SPLITS) const;

  void optbisect(point_t const& ray_origin,
                 std::size_t fx,
                 std::size_t x,
                 bool& intersects,
                 std::size_t& iters,
                 interval<value_type> const& roots_parameter_range =
                     interval<value_type>(0, 1),
                 std::size_t max_iters = MAX_BINARY_SPLITS) const;

  value_type minimum(std::size_t dimension) const;
  value_type maximum(std::size_t dimension) const;

  // check condtions
  bool is_constant(std::size_t dimension, value_type tolerance = 0) const;

  bool is_rational() const;

  bool is_increasing(std::size_t dimension) const;

  // transformations
  void elevate();
  void translate(point_t const& t);

  // bounding volume
  void bbox_simple(axis_aligned_boundingbox<point_t>& bb) const;
  void bbox_tight(axis_aligned_boundingbox<point_t>& bb) const;

  // iterator interface for control points
  point_iterator begin();
  point_iterator end();
  const_point_iterator begin() const;
  const_point_iterator end() const;

  // access to starting and endpoint
  point_t& front();
  point_t& back();
  point_t const& front() const;
  point_t const& back() const;

  // access by subscript operator
  point_t& operator[](std::size_t);
  point_t const& operator[](std::size_t) const;

  // invert curves point order
  void invert();

  // is dim-coordinate of curve's end point higher than its starting point?
  void reverse();

  void intersect_chull(unsigned def,
                       unsigned val,
                       value_type& t0,
                       value_type& t1);

  // reset curve
  void clear();

 private:  // members

  std::vector<point_t> _points;  // control points
  std::size_t _deg;              // degree of curve
};

template <typename point_t>
    std::ostream& operator<<(std::ostream& os, beziercurve<point_t> const& rhs);

typedef beziercurve<point2f> beziercurve2f;
typedef beziercurve<point2d> beziercurve2d;
typedef beziercurve<point3f> beziercurve3f;
typedef beziercurve<point3d> beziercurve3d;

}  // namespace tml

#include "implementation/beziercurve_impl.hpp"

#endif  // TML_BEZIERCURVE_HPP
