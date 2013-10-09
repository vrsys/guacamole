/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : copy_if.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_COPY_IF_HPP
#define TML_COPY_IF_HPP

namespace tml {
namespace util {

template <typename input_iterator, typename output_iterator, typename pred>
void copy_if(input_iterator beg,
             input_iterator end,
             output_iterator beg2,
             pred p) {
  while (beg != end) {
    if (p(*beg)) {
      *beg2++ = *beg;
    }
    ++beg;
  }
}

}  // namespace util
}  // namespace tml

#endif
