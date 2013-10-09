/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : interval_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_INTERVAL_IMPL_HPP
#define TML_INTERVAL_IMPL_HPP

// header, system

namespace tml {

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
interval<value_t>::interval()
    : _min(0),
      _max(0),
      _lower_boundary_type(excluded),
      _upper_boundary_type(excluded) {}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
interval<value_t>::interval(interval const& i)
    : _min(i._min),
      _max(i._max),
      _lower_boundary_type(i._lower_boundary_type),
      _upper_boundary_type(i._upper_boundary_type) {}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
/* explicit */ interval<value_t>::interval(value_t const& a,
                                           value_t const& b,
                                           enum boundary_type lower,
                                           enum boundary_type upper)
    : _min(a > b ? b : a),
      _max(a > b ? a : b),
      _lower_boundary_type(lower),
      _upper_boundary_type(upper) {
  // swizzle boundary conditions
  if (a > b) {
    std::swap(_lower_boundary_type, _upper_boundary_type);
  }
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t> interval<value_t>::~interval() {}

///////////////////////////////////////////////////////////////////////////
template <typename value_t> value_t const interval<value_t>::length() const {
  return _max - _min;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
bool interval<value_t>::overlap(interval const& rhs) const {
  bool result =
      ((rhs.in(_min)) || (rhs.in(_max)) || (in(rhs._min)) || (in(rhs._max)));

  // special case: interval is itself
  result |= (*this == rhs) && !nil();

  // if intervals "touch" each other connection must be in both intervals!
  result &= !(_min == rhs._max && (_lower_boundary_type != included ||
                                   rhs._upper_boundary_type != included));

  result &= !(_max == rhs._min && (_upper_boundary_type != included ||
                                   rhs._lower_boundary_type != included));

  return result;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
value_t interval<value_t>::distance(value_t const& rhs) const {
  return in(rhs)
             ? value_t(0)
             : std::max(value_t(0),
                        std::min(std::fabs(rhs - _min), std::fabs(rhs - _max)));
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
value_t interval<value_t>::distance(interval const& rhs) const {
  return (in(rhs._max) && in(rhs._min)) || (rhs.in(_max) && rhs.in(_min))
             ? value_t(0) /* interval fully included */
             : std::min(distance(rhs._min), distance(rhs._max));
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t> bool interval<value_t>::in(value_t const& a) const {
  if (_lower_boundary_type == included && _upper_boundary_type == included) {
    return (a >= _min && a <= _max);
  }

  if (_lower_boundary_type == excluded && _upper_boundary_type == included) {
    return (a > _min && a <= _max);
  }

  if (_lower_boundary_type == included && _upper_boundary_type == excluded) {
    return (a >= _min && a < _max);
  }

  if (_lower_boundary_type == excluded && _upper_boundary_type == excluded) {
    return (a > _min && a < _max);
  }

  // should not be reached
  return false;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
bool interval<value_t>::in(interval const& other) const {
  if (_lower_boundary_type == included && _upper_boundary_type == included) {
    return (other._min >= _min && other._max <= _max);
  }

  if (_lower_boundary_type == excluded && _upper_boundary_type == included) {
    return (other._min > _min && other._max <= _max);
  }

  if (_lower_boundary_type == included && _upper_boundary_type == excluded) {
    return (other._min >= _min && other._max < _max);
  }

  if (_lower_boundary_type == excluded && _upper_boundary_type == excluded) {
    return (other._min > _min && other._max < _max);
  }

  // should not be reached
  return false;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t> value_t interval<value_t>::center() const {
  return (_max + _min) / 2;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t> value_t interval<value_t>::minimum() const {
  return _min;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t> value_t interval<value_t>::maximum() const {
  return _max;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
boundary_type interval<value_t>::lower_boundary_type() const {
  return _lower_boundary_type;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
boundary_type interval<value_t>::upper_boundary_type() const {
  return _upper_boundary_type;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t> void interval<value_t>::extend(value_t const& d) {
  assert(d >= 0);
  _min -= d;
  _max += d;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
void interval<value_t>::extend_min(value_t const& d) {
  assert(d >= 0);
  _min -= d;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
void interval<value_t>::extend_max(value_t const& d) {
  assert(d >= 0);
  _max += d;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
bool interval<value_t>::greater(value_t const& v) const {
  if (_upper_boundary_type == included) {
    return v > _max;
  } else {
    return v >= _max;
  }
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
bool interval<value_t>::less(value_t const& v) const {
  if (_lower_boundary_type == included) {
    return v < _min;
  } else {
    return v <= _min;
  }
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t> void interval<value_t>::merge(interval const& rhs) {
  if (rhs._min < _min ||
      (rhs._min == _min && rhs._lower_boundary_type == excluded)) {
    _min = rhs._min;
    _lower_boundary_type = rhs._lower_boundary_type;
  }
  if (rhs._max > _max ||
      (rhs._max == _max && rhs._upper_boundary_type == excluded)) {
    _max = rhs._max;
    _upper_boundary_type = rhs._upper_boundary_type;
  }
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t> bool interval<value_t>::nil() const {
  return (_min == _max && (_upper_boundary_type == excluded ||
                           _lower_boundary_type == excluded));
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
void interval<value_t>::print(std::ostream& os) const {
  if (_lower_boundary_type == excluded) {
    os << "(" << _min << " , ";
  } else {
    os << "[" << _min << " , ";
  }
  if (_upper_boundary_type == excluded) {
    os << _max << ")";
  } else {
    os << _max << "]";
  }
}

///////////////////////////////////////////////////////////////////////////
// externals
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
    bool operator<(interval<value_t> const& lhs, interval<value_t> const& rhs) {
  return (lhs.maximum() == rhs.minimum() &&
          (lhs.upper_boundary_type() == excluded ||
           rhs.lower_boundary_type() == excluded)) ||
         (lhs.maximum() < rhs.minimum());
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
    bool operator>(interval<value_t> const& lhs, interval<value_t> const& rhs) {
  return (lhs.minimum() == rhs.maximum() &&
          (lhs.lower_boundary_type() == excluded ||
           rhs.upper_boundary_type() == excluded)) ||
         (lhs.minimum() > rhs.maximum());
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
    bool operator==(interval<value_t> const& lhs,
                    interval<value_t> const& rhs) {
  return (lhs.minimum() == rhs.minimum() && lhs.maximum() == rhs.maximum() &&
          lhs.lower_boundary_type() == rhs.lower_boundary_type() &&
          lhs.upper_boundary_type() == rhs.upper_boundary_type());
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
    bool operator!=(interval<value_t> const& lhs,
                    interval<value_t> const& rhs) {
  return !(lhs == rhs);
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
    interval<value_t> operator+(interval<value_t> const& lhs,
                                interval<value_t> const& rhs) {
  interval<value_t> tmp(lhs);
  tmp.merge(rhs);
  return tmp;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_t>
    std::ostream& operator<<(std::ostream& os,
                             tml::interval<value_t> const& rng) {
  rng.print(os);
  return os;
}
}  // namespace tml

#endif  // TML_INTERVAL_IMPL_HPP
