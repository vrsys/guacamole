/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : beziercurve_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_BEZIERCURVE_IMPL_HPP
#define TML_BEZIERCURVE_IMPL_HPP

// header i/f

// header, system
#include <iterator>   // std::ostream_iterator
#include <algorithm>  // std::reverse
#include <string>     // std::string

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/horner.hpp>

#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox.hpp>
#include <gua/renderer/nurbs_geometry/tml/interval.hpp>

#include <gua/renderer/nurbs_geometry/tml/binomial_coefficient.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/minimal_coordinates.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/maximal_coordinates.hpp>

namespace tml {

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
beziercurve<point_t>::beziercurve()
    : _points(), _deg(0) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
beziercurve<point_t>::beziercurve(beziercurve<point_t> const& cpy)
    : _points(cpy._points), _deg(cpy._deg) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
beziercurve<point_t>::beziercurve(std::size_t order)
    : _points(order), _deg(order - 1) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
beziercurve<point_t>::beziercurve(iterator_t beg, iterator_t end)
    : _points(), _deg(0) {
  std::copy(beg, end, std::back_inserter(_points));
  _deg = _points.size() - 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename container_t>
beziercurve<point_t>::beziercurve(container_t const& c)
    : _points(c.begin(), c.end()), _deg(c.size() - 1) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> beziercurve<point_t>::~beziercurve() {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::swap(beziercurve<point_t>& swp) {
  std::swap(_points, swp._points);
  std::swap(_deg, swp._deg);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziercurve<point_t>& beziercurve<point_t>::operator=(
    beziercurve<point_t> const& cpy) {
  beziercurve tmp(cpy);
  swap(tmp);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::add(point_t const& point) {
  _points.push_back(point);
  _deg = std::size_t(_points.size() - 1);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename external_point_type>
void beziercurve<point_t>::add(external_point_type const& point) {
  add(point_t(point));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline point_t beziercurve<point_t>::evaluate(
    value_type const& t,
    evaluator<point_t> const& eval) const {
  point_t p;
  eval(_points.begin(), _deg + 1, t, p);
  return p;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::evaluate(
    value_type const& t,
    point_t& point,
    evaluator<point_t> const& eval) const {
  eval(_points.begin(), _deg + 1, t, point);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::evaluate(
    value_type const& t,
    point_t& point,
    point_t& dt,
    evaluator<point_t> const& eval) const {
  eval(_points.begin(), _deg + 1, t, point, dt);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziercurve<point_t>::degree() const {
  return _deg;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziercurve<point_t>::order() const {
  return _deg + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::print(std::ostream& os,
                                        std::string const& add) const {
  os << "degree " << _deg << " beziercurve : ";
  std::copy(
      _points.begin(), _points.end(), std::ostream_iterator<point_t>(os, ", "));
  os << add;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziercurve<point_t>::value_type
beziercurve<point_t>::curvature() const {
  if (_deg < 2) {
    return 0;
  } else {
    // compute angles between control points of control polygon as heuristic
    value_type alpha = 0;
    for (std::size_t i = 0; i != _deg; ++i) {
      for (std::size_t j = i + 1; j != _deg; ++j) {
        point_t d0 = (_points[i + 1] - _points[i]);
        point_t d1 = (_points[j + 1] - _points[j]);
        d0.normalize();
        d1.normalize();
        alpha = std::max(alpha,
                         value_type(1) - std::max(value_type(0), dot(d0, d1)));
      }
    }
    return alpha;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziercurve<point_t> beziercurve<point_t>::scaled_hodograph() const {
  beziercurve hodograph(2 * _deg - 1);

  for (std::size_t k = 0; k <= 2 * _deg - 2; ++k) {
    std::size_t e_idx = std::size_t(k / 2);
    std::size_t s_idx = std::max(0, int(k) - int(_deg) + 1);

    for (std::size_t i = s_idx; i <= e_idx; ++i) {
      value_type alpha = value_type(k - 2 * i + 1) *
                         value_type(binomial_coefficient(_deg, i)) *
                         value_type(binomial_coefficient(_deg, k - i + 1));
      alpha = alpha / value_type(binomial_coefficient(2 * _deg - 2, k));

      point_t D_ik = (_points[i].weight() * _points[k - i + 1].weight()) *
                     (_points[k - i + 1] - _points[i]);
      hodograph._points[k] = hodograph._points[k] + alpha * D_ik;
    }
  }

  // set weights to 1
  for (point_iterator i = hodograph._points.begin();
       i < hodograph._points.end();
       ++i) {
    (*i).weight(1);
  }

  return hodograph;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziercurve<point_t> beziercurve<point_t>::hodograph() const {
  beziercurve hodograph;

  for (std::size_t k = 0; k <= _deg - 1; ++k) {
    point_t cp = value_type(_deg) * (_points[k + 1] - _points[k]);
    // set weight back to 1
    cp.weight(1);
    hodograph._points.push_back(cp);
  }
  hodograph._deg = _deg - 1;

  return hodograph;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziercurve<point_t>::beziercurve2_type
beziercurve<point_t>::nishita(std::size_t coord) const {
  beziercurve2_type nishita_transformation;

  for (std::size_t i = 0; i < _points.size(); ++i) {
    point<value_type, 2> equidistant_control_point(
        value_type(i) / value_type(_deg), /* equidistant on first coordinate*/
        _points[i][coord],                /* value on y axis */
        _points[i].weight());
    /* keep weight, although unspecified for rational curves*/
    nishita_transformation.add(equidistant_control_point);
  }

  return nishita_transformation;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline polynomial<typename beziercurve<point_t>::value_type>
beziercurve<point_t>::as_polynomial(std::size_t coord) const {
  polynomial<value_type> fx;
  std::size_t n = degree();

  for (std::size_t i = 0; i != order(); ++i) {
    polynomial<value_type> t(0, 1);
    polynomial<value_type> one_minus_t(1, -1);

    fx += _points[i][coord] * value_type(tml::binomial_coefficient(n, i)) *
          tml::pow(t, i) * tml::pow(one_minus_t, n - i);
  }

  return fx;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::clip_left(value_type t) {
  assert(t >= 0.0 && t <= 1.0);

  for (point_iterator i = _points.begin(); i < _points.end(); ++i) {
    *i = i->as_homogenous();
  }

  // helper
  beziercurve HP(order());

  HP._points[_deg] = _points[_deg];

  for (std::size_t i = 0; i <= _deg; ++i) {
    for (std::size_t j = 0; j < _deg - i; ++j) {
      _points[j] = value_type(1.0 - t) * _points[j] + t * _points[j + 1];
    }
    if (i < _deg) {
      HP._points[_deg - i] = _points[_deg - i];
    }
  }

  HP._points[0] = _points[0];

  for (point_iterator i = HP._points.begin(); i < HP._points.end(); ++i) {
    (*i) = (*i).as_euclidian();
  }

  // swap with helper polygon
  swap(HP);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::clip_right(value_type t) {
  assert(t >= 0.0 && t <= 1.0);

  for (point_iterator i = _points.begin(); i < _points.end(); ++i) {
    *i = i->as_homogenous();
  }

  // helper
  beziercurve HP(order());
  HP._points[0] = _points[0];

  for (std::size_t i = 0; i <= _deg; ++i) {
    for (std::size_t j = 0; j < _deg - i; ++j) {
      _points[j] = value_type(1.0 - t) * _points[j] + t * _points[j + 1];
    }
    if (i < _deg) {
      HP._points[i + 1] = _points[0];
    }
  }

  for (point_iterator i = HP._points.begin(); i < HP._points.end(); ++i) {
    (*i) = (*i).as_euclidian();
  }

  // swap with helper polygon
  swap(HP);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::clip_lr(value_type l, value_type r) {
  if (l < r) {
    clip_left(l);
    clip_right((r - l) / (value_type(1.0) - l));
  } else {
    clip_left(r);
    clip_right((l - r) / (value_type(1.0) - r));
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::split(value_type t,
                                        beziercurve& left,
                                        beziercurve& right) const {
  left = *this;
  right = *this;

  left.clip_right(t);
  right.clip_left(t);

  // assert continuity
  left.back() = right.front();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename parameter_container_t, typename curve_container_t>
inline void beziercurve<point_t>::split(
    parameter_container_t const& ordered_params,
    curve_container_t& curvesegments) const {
  std::vector<value_type> params(ordered_params.begin(), ordered_params.end());
  point_t connection;

  switch (ordered_params.size()) {
    case 0:  // no split
             {
      curvesegments.push_back(*this);
      break;
    }

    case 1:  // one split
             {
      beziercurve<point_t> left, right;
      split(params.front(), left, right);
      curvesegments.push_back(left);
      curvesegments.push_back(right);
      break;
    }
    default:  // multiple splits
              {
      for (std::size_t i = 0; i != params.size(); ++i) {

        // first segment
        if (i == 0) {
          beziercurve<point_t> tmp(*this);
          tmp.clip_right(params.at(i));  // clip right part of tmp copy
          connection = tmp.back();       // save last point for continuity
          curvesegments.push_back(tmp);  // store first curve segment
        }

        // last segment
        if (i == params.size() - 1) {
          beziercurve<point_t> tmp(*this);
          tmp.clip_left(params.at(i));  // clip left part of tmp copy
          tmp.front() =
              connection;  // set first point of segment to last point of
                           // preceding segment
          curvesegments.push_back(tmp);  // store last curve segment
        }

        //inner segments
        if (i != params.size() - 1) {
          beziercurve<point_t> tmp(*this);
          tmp.clip_lr(params.at(i),
                      params.at(i + 1));  // clip left and right part
          tmp.front() =
              connection;  // set first point of segment to last point of
                           // preceding segment
          connection = tmp.back();       // save last point for continuity
          curvesegments.push_back(tmp);  // store inner segment
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::extrema(std::size_t dim,
                                          std::set<value_type>& roots,
                                          std::size_t iterations) const {
  // linear curves have no local extrema
  if (1 >= _deg || weak_monotonic(dim)) {
    return;
  }

  // otherwise search for extrema
  beziercurve hodo;
  if (is_rational()) {
    hodo = scaled_hodograph();
  } else {
    hodo = hodograph();
  }

  // insert explicit distance curve
  beziercurve2_type hodo2D = hodo.nishita(dim);

  std::vector<beziercurve2_type> nth_derivative_curves;
  nth_derivative_curves.push_back(hodo2D);  // push first derivative

  // derive(hodograph) and transform each time into nishita transformation
  for (std::size_t i = 1; i < hodo.degree(); ++i) {
    beziercurve2_type derivative_i =
        nth_derivative_curves.back().hodograph().nishita(point_t::v);
    nth_derivative_curves.push_back(derivative_i);
  }

  // only roots between 0.0 and 1.0 possible
  value_type umin(0);
  value_type umax(1);

  std::vector<interval<value_type> > intervals;
  intervals.push_back(interval<value_type>(0, 1));

  for (int i = int(nth_derivative_curves.size()) - 1; i >= 0; --i) {
    value_type last_root = umin;
    std::vector<interval<value_type> > new_intervals;

    for (std::size_t v = 0; v < intervals.size(); ++v) {
      value_type root = 0;
      bool isroot = false;

      // due to nishita transformation the concerning direction is v
      nth_derivative_curves[i]
          .bisect(point_t::v, 0.0, isroot, root, intervals[v], iterations);

      if (isroot) {
        new_intervals.push_back(interval<value_type>(last_root, root));
        last_root = root;
      }

      // roots of final curve
      if (isroot && i == 0) {
        roots.insert(value_type(root));
      }
    }
    new_intervals.push_back(interval<value_type>(last_root, umax));
    std::swap(intervals, new_intervals);
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
bool beziercurve<point_t>::weak_monotonic(std::size_t dim) const {
  if (_points.size() < 3) {
    return true;
  } else {
    bool increasing = true;
    bool decreasing = true;

    for (unsigned int i = 0; i != _points.size() - 1; ++i) {
      increasing &= _points[i][dim] >= _points[i + 1][dim];
      decreasing &= _points[i][dim] <= _points[i + 1][dim];
    }

    return increasing || decreasing;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline bool beziercurve<point_t>::assert_extremum(std::size_t dim,
                                                  value_type const& t,
                                                  value_type const& dt) const {
  point_t p0 = evaluate(std::max(t - dt, value_type(0)));
  point_t pt = evaluate(t);
  point_t p1 = evaluate(std::min(t + dt, value_type(1)));

  return (pt[dim] > p0[dim] && pt[dim] > p1[dim]) ||   /* maxima */
             (pt[dim] < p0[dim] && pt[dim] < p1[dim]); /* minima */
}

//////////////////////////////////////////////////////////////////////////////
// result for non-monotonic curve is unspecified
//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::bisect(std::size_t dim,
                                         value_type const& value,
                                         bool& isroot,
                                         value_type& root,
                                         interval<value_type> const& para_rng,
                                         std::size_t iterations) const {
  // it is assumed that value v is between front[v] and back[v]
  /*assert((value >= front()[dim] && value <= back()[dim]) ||
         (value <= front()[dim] && value >= back()[dim]));*/

  // handle special cases
  if (front()[dim] == value && para_rng.minimum() == 0) {
    isroot = true;
    root = 0;
    return;
  }

  if (back()[dim] == value && para_rng.maximum() == 1) {
    isroot = true;
    root = 1;
    return;
  }

  // if no special case
  point_t pmin;
  point_t pmax;

  value_type umin = para_rng.minimum();
  value_type umax = para_rng.maximum();

  // get min and max values
  evaluate(para_rng.minimum(), pmin, horner<point_t>());
  evaluate(para_rng.maximum(), pmax, horner<point_t>());

  // set default values
  isroot = false;

  bool intersects_increasing = (pmin[dim] < value) && (pmax[dim] > value);
  bool intersects_decreasing = (pmin[dim] > value) && (pmax[dim] < value);

  if (intersects_increasing || intersects_decreasing) {
    isroot = true;  // there must be a root
    for (std::size_t i = 0; i <= iterations; ++i) {
      root = (umin + umax) / value_type(2.0);
      point_t pi;
      evaluate(root, pi, horner<point_t>());

      // reset limits for increasing curve
      if (intersects_increasing) {
        if (pi[dim] > value) {
          umax = root;
        } else {
          umin = root;
        }
      }

      // reset limits for decreasign curve
      if (intersects_decreasing) {
        if (pi[dim] > value) {
          umin = root;
        } else {
          umax = root;
        }
      }
    }
  }

  root = (umin + umax) / value_type(2.0);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::optbisect(
    point_t const& point,
    std::size_t fx,
    std::size_t x,
    bool& intersects,
    std::size_t& iters,
    interval<value_type> const& para_rng,
    std::size_t max_iters) const {
  point_t pmin;
  point_t pmax;

  if (para_rng.minimum() != 0.0) {
    evaluate(para_rng.minimum(), pmin, horner<point_t>());
  } else {
    pmin = _points.front();
  }

  if (para_rng.maximum() != 1.0) {
    evaluate(para_rng.maximum(), pmax, horner<point_t>());
  } else {
    pmax = _points.back();
  }

  value_type umin = para_rng.minimum();
  value_type umax = para_rng.maximum();

  // no intersection if start and end point do not differ in sign
  if (!(interval<value_type>(pmin[fx], pmax[fx], tml::included, tml::included)
            .in(point[fx]))) {
    iters = 0;
    intersects = false;
    return;
  }

  bool inc_dx(pmin[x] < pmax[x]);
  bool inc_dy(pmin[fx] < pmax[fx]);

  point_t pi;
  for (std::size_t i = 0; i <= max_iters; ++i) {
    value_type root = (umin + umax) / value_type(2.0);
    evaluate(root, pi, horner<point_t>());

    if (point[x] < pi[x]) {  // check early intersect
      if ((inc_dx && inc_dy && point[fx] > pi[fx]) ||
          (!inc_dx && inc_dy && point[fx] < pi[fx]) ||
          (inc_dx && !inc_dy && point[fx] < pi[fx]) ||
          (!inc_dx && !inc_dy && point[fx] > pi[fx])) {
        intersects = true;
        iters = i + 1;
        return;
      }

    } else {  // check early non-intersect
      if ((inc_dx && inc_dy && point[fx] < pi[fx]) ||
          (!inc_dx && inc_dy && point[fx] > pi[fx]) ||
          (inc_dx && !inc_dy && point[fx] > pi[fx]) ||
          (!inc_dx && !inc_dy && point[fx] < pi[fx])) {
        intersects = false;
        iters = i + 1;
        return;
      }
    }

    // continue bisection
    if (pi[fx] > point[fx]) {
      umax = root;
    } else {
      umin = root;
    }
  }
  iters = max_iters;
  if (pi[x] > point[x]) {
    intersects = true;
  } else {
    intersects = false;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type beziercurve<point_t>::minimum(
    std::size_t dim) const {
  minimal_coordinates<point_t> res = std::for_each(
      _points.begin(), _points.end(), minimal_coordinates<point_t>(dim));
  return res.result();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type beziercurve<point_t>::maximum(
    std::size_t dim) const {
  maximal_coordinates<point_t> res = for_each(
      _points.begin(), _points.end(), maximal_coordinates<point_t>(dim));
  return res.result();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline bool beziercurve<point_t>::is_constant(std::size_t dim,
                                              value_type tolerance) const {
  if (!_points.empty()) {
    if (tolerance == value_type(0)) {
      bool is_lin(true);
      value_type compare = _points.front()[dim];
      for (std::size_t i = 0; i != _points.size(); ++i) {
        is_lin &= bool(compare == _points[i][dim]);
        compare = _points[i][dim];
      }
      return is_lin;
    } else {
      return (std::fabs(this->maximum(dim) - this->minimum(dim)) <= tolerance);
    }
  } else {
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline bool beziercurve<point_t>::is_rational() const {
  for (std::size_t i = 0; i < _points.size(); ++i) {
    if (_points[i].weight() != value_type(1))
      return true;
  }
  return false;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline bool beziercurve<point_t>::is_increasing(std::size_t dim) const {
  return (_points.back()[dim] > _points.front()[dim]);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void beziercurve<point_t>::elevate() {
  // generate new control polygon with _deg+2 control points
  std::vector<point_t> new_cp;

  if (_points.size() > 1) {
    // copy first point
    new_cp.push_back(_points.front());

    // generate new inner points
    for (std::size_t i = 1; i < _points.size(); ++i) {
      value_type alpha = value_type(i) / value_type(_deg + 1);
      point_t p_in_homogen =
          alpha * _points[i - 1].as_homogenous() +
          value_type(1.0 - alpha) * _points[i].as_homogenous();
      new_cp.push_back(p_in_homogen.as_euclidian());
    }

    // copy last point
    new_cp.push_back(_points.back());

    // swap control points and elevate degree
    std::swap(new_cp, _points);
    ++_deg;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::translate(point_t const& t) {
  for (point_iterator i = _points.begin(); i != _points.end(); ++i) {
    for (std::size_t axis = 0; axis != point_t::coordinates; ++axis) {
      (*i)[axis] += t[axis];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::bbox_simple(
    axis_aligned_boundingbox<point_t>& bbox) const {
  assert(order() > 0);

  point_t pmin, pmax;

  for (std::size_t axis = 0; axis != point_t::coordinates; ++axis) {
    pmin[axis] = this->minimum(axis);
    pmax[axis] = this->maximum(axis);
  }

  bbox = axis_aligned_boundingbox<point_t>(pmin, pmax);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::bbox_tight(
    axis_aligned_boundingbox<point_t>& bbox) const {
  std::set<value_type> expnts;

  // extrema in curve
  for (std::size_t axis = 0; axis != point_t::coordinates; ++axis) {
    extrema(axis, expnts, 64);
  }

  // extrema at endpoints
  expnts.insert(value_type(0));
  expnts.insert(value_type(1));

  beziercurve<point_t> extrempoint_curve;

  for (typename std::set<value_type>::const_iterator i = expnts.begin();
       i != expnts.end();
       ++i) {
    point_t extrema;
    evaluate(*i, extrema, horner<point_t>());
    extrempoint_curve.add(extrema);
  }

  extrempoint_curve.bbox_simple(bbox);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename std::vector<point_t>::iterator beziercurve<point_t>::begin() {
  return _points.begin();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename std::vector<point_t>::iterator beziercurve<point_t>::end() {
  return _points.end();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename std::vector<point_t>::const_iterator
beziercurve<point_t>::begin() const {
  return _points.begin();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename std::vector<point_t>::const_iterator
beziercurve<point_t>::end() const {
  return _points.end();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline point_t const& beziercurve<point_t>::front() const {
  return _points.front();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline point_t const& beziercurve<point_t>::back() const {
  return _points.back();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline point_t& beziercurve<point_t>::front() {
  return _points.front();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline point_t& beziercurve<point_t>::back() {
  return _points.back();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline point_t& beziercurve<point_t>::operator[](std::size_t i) {
  return _points[i];
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline point_t const& beziercurve<point_t>::operator[](std::size_t i) const {
  return _points[i];
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void beziercurve<point_t>::invert() {
  std::vector<point_t> tmp;
  std::copy(_points.rbegin(), _points.rend(), std::back_inserter(tmp));
  std::swap(tmp, _points);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void beziercurve<point_t>::reverse() {
  std::reverse(_points.begin(), _points.end());
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziercurve<point_t>::intersect_chull(unsigned def,
                                                  unsigned val,
                                                  value_type& t1,
                                                  value_type& t2) {
#if GPUCAST_INLINE_BOOST_USAGE
  value_type mn = boost::numeric::bounds<value_type>::highest();
  value_type mx = boost::numeric::bounds<value_type>::lowest();
#else
  value_type mn = std::numeric_limits<value_type>::max();
  value_type mx = -std::numeric_limits<value_type>::max();
#endif

  for (const_point_iterator i = _points.begin(); i != _points.end(); ++i) {
    for (const_point_iterator j = i + 1; j != _points.end(); ++j) {
      // copy points
      point_t fst = (*i);
      point_t snd = (*j);

      // find intersections
      if ((fst[val] < -std::numeric_limits<value_type>::epsilon() &&
           snd[val] > std::numeric_limits<value_type>::epsilon()) ||
          (fst[val] > std::numeric_limits<value_type>::epsilon() &&
           snd[val] < -std::numeric_limits<value_type>::epsilon())) {
        value_type dx = (snd[val] - fst[val]) / (snd[def] - fst[def]);
        value_type xi = ((dx * fst[def] - fst[val]) * (snd[def] - fst[def])) /
                        (snd[val] - fst[val]);

        mn = std::min(mn, xi);
        mx = std::max(mx, xi);
      }

      // find points on axis
      if (std::abs(fst[val]) <= std::numeric_limits<value_type>::epsilon()) {
        if ((*i)[def] < mn)
          mn = (*i)[def];
        if ((*i)[def] > mx)
          mx = (*i)[def];
      }
    }
  }
  t1 = mn;
  t2 = mx;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void beziercurve<point_t>::clear() {
  _points.clear();
  _deg = 0;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
    inline std::ostream& operator<<(std::ostream& os,
                                    beziercurve<point_t> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml

#endif  // TML_BEZIERCURVE_IMPL_HPP
