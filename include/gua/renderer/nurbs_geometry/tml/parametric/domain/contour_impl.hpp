/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : contour_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour_segment.hpp>

namespace tml {

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
template <typename curve_ptr_iterator_t>
contour<value_t>::contour(curve_ptr_iterator_t begin, curve_ptr_iterator_t end)
    : _curves(begin, end) {}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> contour<value_t>::~contour() {}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> bool contour<value_t>::valid() const {
  if (_curves.empty()) {
    return false;
  } else {
    curve_ptr const& c0 = _curves.front();
    curve_ptr const& cn = _curves.back();

    // check if curve is closed
    if (c0->front() != cn->back()) {
      return false;
    }

    // iterate all points to check if curve is piecewise continous
    point_type const& current = c0->back();
    BOOST_FOREACH(curve_ptr const & c, _curves) {
      if (c != c0) {
        if (current != c->front()) {
          return false;
        } else {
          current = c->back();
        }
      }
    }
  }

  // past all checks -> valid
  return true;
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> bool contour<value_t>::empty() const {
  return _curves.empty();
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> std::size_t contour<value_t>::size() const {
  return _curves.size();
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
typename contour<value_t>::const_curve_iterator
contour<value_t>::begin() const {
  return _curves.begin();
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
typename contour<value_t>::const_curve_iterator contour<value_t>::end() const {
  return _curves.end();
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
template <typename contour_segment_ptr_container>
void contour<value_t>::monotonize(contour_segment_ptr_container& target) const {
  std::vector<curve_type> contour_of_bimonotonic_curves;

  // split contour at all extremas
  BOOST_FOREACH(curve_ptr const & c, _curves) {
    // identify extrema
    std::set<value_type> t_extrema;
    c->extrema(point_type::u, t_extrema);
    c->extrema(point_type::v, t_extrema);

    // split at extrema if there are any - else push original curve
    if (t_extrema.empty()) {
      contour_of_bimonotonic_curves.push_back(*c);
    } else {
      c->split(t_extrema, contour_of_bimonotonic_curves);
    }
  }

  // trace contour - consisting of bimonotonic curve segments
  std::vector<curve_ptr> current_contour;

  bool contour_restart = true;
  bool u_increasing = false;
  bool v_increasing = false;

  BOOST_FOREACH(curve_type const & curve, contour_of_bimonotonic_curves) {
    // start first bi-monotonic contour
    if (contour_restart) {
      contour_restart = false;
      u_increasing = curve.is_increasing(point_type::u);
      v_increasing = curve.is_increasing(point_type::v);
      current_contour.push_back(curve_ptr(new curve_type(curve)));
    } else {
      // if curve has same monotony as contour -> add curve to current contour
      if (u_increasing == curve.is_increasing(point_type::u) &&
          v_increasing == curve.is_increasing(point_type::v) &&
          !curve.is_constant(point_type::u) &&
          !curve.is_constant(point_type::v)) {
        current_contour.push_back(curve_ptr(new curve_type(curve)));
      } else {  // push previous contour_segment and restart new contour_segment
        target.push_back(contour_segment_ptr(new contour_segment_type(
            current_contour.begin(), current_contour.end())));
        current_contour.clear();
        u_increasing = curve.is_increasing(point_type::u);
        v_increasing = curve.is_increasing(point_type::v);
        current_contour.push_back(curve_ptr(new curve_type(curve)));
      }
    }
  }

  // flush last contour
  if (!current_contour.empty()) {
    target.push_back(contour_segment_ptr(new contour_segment_type(
        current_contour.begin(), current_contour.end())));
  }
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour<value_t>::print(std::ostream& os) const {
  os << "contour with " << _curves.size() << " curves :" << std::endl;
  std::for_each(
      _curves.begin(),
      _curves.end(),
      std::bind(&curve_type::print, std::placeholders::_1, std::ref(os)));
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
    std::ostream& operator<<(std::ostream& os,
                             tml::contour<value_t> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml
