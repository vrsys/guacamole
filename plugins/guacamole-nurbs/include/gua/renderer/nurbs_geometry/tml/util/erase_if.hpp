/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : erase_if.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_ERASE_IF_HPP
#define TML_ERASE_IF_HPP

// header, system

namespace tml {
namespace util {

template <typename container_type, typename pred>
void erase_if(container_type& c, pred f) {
  typedef typename container_type::iterator iterator;

  iterator x = std::find_if(c.begin(), c.end(), f);
  while (x != c.end()) {
    c.erase(x);
    x = std::find_if(c.begin(), c.end(), f);
  }
}

}  // namespace util
}  // namespace tml

#endif  // TML_ERASE_IF_HPP
