/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : interval.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_INTERVAL_HPP
#define TML_INTERVAL_HPP

// header, system
#include <cmath>
#include <cassert>
#include <iostream>

namespace tml {

// excluded   = borders do NOT belong to interval
// included   = border do belong to interval
enum boundary_type {
  excluded,
  included
};

template <typename value_t> class interval {

 public:  // enums / typedefs

  typedef value_t value_type;

 public:  // c'tor / d'tor

  interval();

  interval(interval const&);

  explicit interval(value_t const& a,
                    value_t const& b,
                    enum boundary_type lower = excluded,
                    enum boundary_type upper = excluded);
  ~interval();

 public:

  value_t const length() const;

  bool overlap(interval const& rhs) const;

  value_t distance(value_t const& rhs) const;
  value_t distance(interval const& rhs) const;

  bool in(value_t const& a) const;
  bool in(interval const& other) const;

  bool greater(value_t const& v) const;
  bool less(value_t const& v) const;

  bool nil() const;

  value_t center() const;
  value_t minimum() const;
  value_t maximum() const;

  boundary_type lower_boundary_type() const;
  boundary_type upper_boundary_type() const;

  // exceed both limits, e.g. (3,4] extended by 1 is (2,5]
  void extend(value_t const& d);

  // exceed minimum, e.g. (2,4) extend_min by 3 is (-1,4)
  void extend_min(value_t const& d);

  // exceed intervals maximum, e.g. (2,4) extend_max by 3 is (2,7)
  void extend_max(value_t const& d);

  // merge two intervals
  // undefined intervals will be included into limits, e.g. [0,1] merged with
  // [3,7) is [0,7)
  void merge(interval const& rhs);

  void print(std::ostream& os) const;

 private:

  value_t _min;
  value_t _max;

  boundary_type _lower_boundary_type;
  boundary_type _upper_boundary_type;

};

template <typename value_t>
    bool operator<(interval<value_t> const& lhs, interval<value_t> const& rhs);

template <typename value_t>
    bool operator>(interval<value_t> const& lhs, interval<value_t> const& rhs);

template <typename value_t>
    bool operator==(interval<value_t> const& lhs, interval<value_t> const& rhs);

template <typename value_t>
    bool operator!=(interval<value_t> const& lhs, interval<value_t> const& rhs);

template <typename value_t>
    interval<value_t> operator+(interval<value_t> const& lhs,
                                interval<value_t> const& rhs);

template <typename value_t>
    std::ostream& operator<<(std::ostream& os,
                             tml::interval<value_t> const& rng);

}  // namespace tml

namespace tml {

typedef interval<double> intervald;
typedef interval<float> intervalf;
typedef interval<int> intervali;
typedef interval<unsigned> intervalu;

}  // namespace tml

#include <gua/renderer/nurbs_geometry/tml/interval_impl.hpp>

#endif  // TML_INTERVAL_HPP
