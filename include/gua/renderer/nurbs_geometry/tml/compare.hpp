/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : compare.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_COMPARE_HPP
#define TML_COMPARE_HPP

// header, system

namespace tml {

template <typename value_type>
bool weak_equal(value_type const& lhs,
                value_type const rhs,
                value_type const& epsilon) {
  return fabs(rhs - lhs) < epsilon;
}

}  // namespace tml

#endif  // TML_COMPARE_HPP
