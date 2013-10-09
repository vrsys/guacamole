/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : previous_next_set.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_PREVIOUS_NEXT_SET
#define TML_PREVIOUS_NEXT_SET

//#include <renderer/nurbs_geometry/tml/util/pair_adaptor.hpp>
#include <boost/iterator/transform_iterator.hpp>

namespace tml {

template <typename ptr_type> struct previous_next_set {
  previous_next_set() : _prev() {}

  void operator()(ptr_type p) {
    if (_prev) {
      _prev->next(p);
      p->previous(_prev);
    }
    _prev = p;
  }

  ptr_type _prev;
};

}  // namespace tml

#endif  // TML_PREVIOUS_NEXT_SET
