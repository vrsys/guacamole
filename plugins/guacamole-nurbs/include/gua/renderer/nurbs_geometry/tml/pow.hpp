/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : pow.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_POW_HPP
#define TML_POW_HPP

// header, system

namespace tml {

template <int base, int exponent> struct meta_pow {
  enum {
    result = base * meta_pow < base,
    exponent - 1 > ::result
  };
};

template <int base> struct meta_pow<base, 0> {
  enum {
    result = 1
  };
};

template <int base> struct meta_pow<base, 1> {
  enum {
    result = base
  };
};

}  // namespace tml

#endif  // TML_POW_HPP
