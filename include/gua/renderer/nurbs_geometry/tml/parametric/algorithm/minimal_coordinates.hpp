/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : minimal_coordinates.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_MINIMAL_COORDINATES_HPP
#define TML_MINIMAL_COORDINATES_HPP

#include <limits>
#include <algorithm>

namespace tml {

template <typename point_t> class minimal_coordinates {
 public:

  typedef typename point_t::value_type value_type;

 public:
  minimal_coordinates(std::size_t coord)
      : _min(std::numeric_limits<value_type>::max()), _coord(coord) {}

  void operator()(point_t const& p) { _min = std::min(_min, p[_coord]); }

  value_type const& result() const { return _min; }

 private:

  value_type _min;
  std::size_t _coord;

};

}  // namespace tml

#endif
