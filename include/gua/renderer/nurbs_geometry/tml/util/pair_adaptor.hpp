/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : pair_adaptor.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_PAIR_ADAPTOR_HPP
#define TML_PAIR_ADAPTOR_HPP

namespace tml {
namespace util {

template <typename pair_type>
typename pair_type::second_type const& second_adaptor(pair_type const& pair) {
  return pair.second;
}

template <typename pair_type>
typename pair_type::first_type const& first_adaptor(pair_type const& pair) {
  return pair.first;
}

}  // namespace util
}  // namespace tml

#endif
