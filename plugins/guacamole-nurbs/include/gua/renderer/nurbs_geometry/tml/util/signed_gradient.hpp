/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : signed_gradient.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_SIGNED_GRADIENT_HPP
#define TML_SIGNED_GRADIENT_HPP

#include <gua/renderer/nurbs_geometry/tml/util/polar_conversion.hpp>

namespace tml {
namespace util {

/////////////////////////////////////////////////////////////////////////////
// given a signed gradient and asssuming that a positive gradient means 'inside'
// this method classifies a given sample point if it also lies 'inside'
/////////////////////////////////////////////////////////////////////////////
template <typename vec2_t> class classify_sample_by_signed_gradient {
 public:
  typedef typename vec2_t::value_type value_type;

  bool operator()(vec2_t const& angle_radius, vec2_t const& sample) const {
    polar_to_euclid<vec2_t> r2e;

    vec2_t gradient = r2e(angle_radius);
    vec2_t normalized_gradient = gradient / gradient.abs();

    value_type distance = dot(normalized_gradient, sample - gradient);

    return (distance < 0) ^ (angle_radius[1] < 0);
  }
};

}  // namespace util
}  // namespace tml

#endif  // TML_SIGNED_GRADIENT_HPP
