/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : binomial_coefficient.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_BINOMIAL_COEFFICIENT_HPP
#define TML_BINOMIAL_COEFFICIENT_HPP

namespace tml {
/**
 * faculty (n) recursive
 */
inline std::size_t factorial_recursive(std::size_t i) {
  return i <= 1 ? 1 : i * factorial_recursive(i - 1);
}

/**
 * faculty (n) iterative
 */
inline std::size_t factorial(std::size_t n) {
  std::size_t fac = 1;

  for (std::size_t i = 1; i <= n; ++i) {
    fac *= i;
  }

  return fac;
}

/**
 * binomial_coefficent (n,i)
 */
inline std::size_t binomial_coefficient(std::size_t n, std::size_t i) {
  return i > n ? 0 : factorial(n) / (factorial(n - i) * factorial(i));
}
}

#endif
