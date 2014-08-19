/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : space_adapter.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_SPACE_ADAPTER_HPP
#define TML_SPACE_ADAPTER_HPP

namespace tml {
template <typename point_t> class point_to_euclid_space_adapter {
 public:
  point_t operator()(point_t const& p) const { return p.as_euclidian(); }
};

template <typename point_t> class point_to_homogenous_space_adapter {
 public:
  point_t operator()(point_t const& p) const { return p.as_homogenous(); }
};

}
#endif
