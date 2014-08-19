/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : concatenate.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_CONCATENATE_HPP
#define TML_CONCATENATE_HPP

// header, system

namespace tml {
namespace util {

template <typename insert_iterator_t,
          typename input_iterator_a,
          typename input_iterator_b>
void concatenate(input_iterator_a begin0,
                 input_iterator_a end0,
                 input_iterator_b begin1,
                 input_iterator_b end1,
                 insert_iterator_t begin2) {
  std::copy(begin1, end1, std::copy(begin0, end0, begin2));
}

}  // namespace util
}  // namespace tml

#endif  // TML_CONCATENATE_HPP
