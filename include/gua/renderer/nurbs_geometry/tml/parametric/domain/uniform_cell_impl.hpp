/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : uniform_cell.hpp
*  project    : tml
*  description:
*
********************************************************************************/
// includes, system

// includes, project
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/vertical_interval.hpp>

namespace tml {

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
uniform_cell<value_type>::uniform_cell()
    : _parent(),
      _horizontal_interval(0, 0),
      _vertical_interval(0, 0),
      _area(0),
      _previous(),
      _next(),
      _intersections(0),
      _curves_near_cell() {}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
uniform_cell<value_type>::uniform_cell(vertical_interval_ptr parent,
                                       interval_type const& horizontal_interval,
                                       interval_type const& vertical_interval)
    : _parent(parent),
      _horizontal_interval(horizontal_interval),
      _vertical_interval(vertical_interval),
      _area(0),
      _previous(),
      _next(),
      _intersections(0),
      _curves_near_cell() {
  _area = _horizontal_interval.length() * _vertical_interval.length();
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
uniform_cell<value_type>::uniform_cell(vertical_interval_ptr const& parent,
                                       interval_type const& horizontal_interval,
                                       interval_type const& vertical_interval,
                                       shared_ptr_type const& prev,
                                       shared_ptr_type const& next)
    : _parent(parent),
      _horizontal_interval(horizontal_interval),
      _vertical_interval(vertical_interval),
      _area(0),
      _previous(prev),
      _next(next),
      _intersections(0),
      _curves_near_cell() {
  _area = _horizontal_interval.length() * _vertical_interval.length();
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type> uniform_cell<value_type>::~uniform_cell() {}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
void uniform_cell<value_type>::next(shared_ptr_type n) {
  _next = n;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
void uniform_cell<value_type>::previous(shared_ptr_type p) {
  _previous = p;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename uniform_cell<value_type>::shared_ptr_type
uniform_cell<value_type>::next() const {
  return _next;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename uniform_cell<value_type>::shared_ptr_type
uniform_cell<value_type>::previous() const {
  return _previous;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
void uniform_cell<value_type>::intersections(std::size_t known_intersections) {
  _intersections = known_intersections;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
std::size_t uniform_cell<value_type>::intersections() const {
  return _intersections;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename uniform_cell<value_type>::interval_type&
uniform_cell<value_type>::get_horizontal_interval() {
  return _horizontal_interval;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename uniform_cell<value_type>::interval_type&
uniform_cell<value_type>::get_vertical_interval() {
  return _vertical_interval;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
void uniform_cell<value_type>::add(curvesegment_ptr const& segment) {
  _curves_near_cell.insert(segment);
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
void uniform_cell<value_type>::remove(curvesegment_ptr const& segment) {
  _curves_near_cell.erase(segment);
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
value_type uniform_cell<value_type>::minimal_distance(
    curvesegment_type const& c) const {
  value_type du = _horizontal_interval.distance(c.delta(point_type::u));
  value_type dv = _vertical_interval.distance(c.delta(point_type::v));

  return std::sqrt(du * du + dv * dv);
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
std::size_t uniform_cell<value_type>::size() const {
  return _curves_near_cell.size();
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type> void uniform_cell<value_type>::clear() {
  _intersections = 0;
  _curves_near_cell.clear();
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename uniform_cell<value_type>::iterator uniform_cell<value_type>::begin() {
  return _curves_near_cell.begin();
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename uniform_cell<value_type>::iterator uniform_cell<value_type>::end() {
  return _curves_near_cell.end();
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename uniform_cell<value_type>::const_iterator
uniform_cell<value_type>::begin() const {
  return _curves_near_cell.begin();
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename uniform_cell<value_type>::const_iterator
uniform_cell<value_type>::end() const {
  return _curves_near_cell.end();
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
void uniform_cell<value_type>::print(std::ostream& os,
                                     std::string const& addinfo) const {
  os << "uniform cell : u = " << _horizontal_interval
     << " , v = " << _vertical_interval << std::endl;
  os << "  - intersections  : " << _intersections << std::endl;
  os << "  - curves to test : " << _curves_near_cell.size() << addinfo;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
    std::ostream& operator<<(std::ostream& os,
                             tml::uniform_cell<value_type> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml
