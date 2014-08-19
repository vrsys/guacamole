/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : determine_dimension_from_endpoints.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_DETERMINE_DIMENSION_FROM_ENDPOINTS
#define TML_DETERMINE_DIMENSION_FROM_ENDPOINTS

// includes, system

namespace tml {

///////////////////////////////////////////////////////////////////////////////
template <typename curve_ptr_type> struct determine_dimension_from_endpoints {
  typedef typename curve_ptr_type::value_type curve_type;
  typedef typename curve_type::point_type point_type;
  typedef typename curve_type::value_type value_type;
  typedef interval<value_type> interval_type;

  determine_dimension_from_endpoints() : initialized(false) {}

  void operator()(curve_ptr_type c) {
    if (initialized) {
      for (std::size_t i = 0; i != point_type::coordinates; ++i) {
        max_interval[i].merge(interval_type(c->front()[i], c->back()[i]));
      }
    } else {
      for (std::size_t i = 0; i != point_type::coordinates; ++i) {
        max_interval[i] = interval_type(c->front()[i], c->back()[i]);
      }
      initialized = true;
    }
  }

  bool initialized;
  interval_type max_interval[point_type::coordinates];
};

}  // namespace tml

#endif  // TML_GENERATE_INTERVALS_FROM_ENDPOINTS_HPP
