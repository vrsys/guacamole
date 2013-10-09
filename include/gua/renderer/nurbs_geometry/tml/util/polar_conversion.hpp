/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : polar_conversion.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_POLAR_CONVERSION_HPP
#define TML_POLAR_CONVERSION_HPP

namespace tml {
namespace util {

/////////////////////////////////////////////////////////////////////////////
// [x,y] -> [alpha, radius]
/////////////////////////////////////////////////////////////////////////////
template <typename vec2_t> class polar_to_euclid {
 public:
  vec2_t operator()(vec2_t const& angle_radius) const {
    return vec2_t(angle_radius[1] * cos(angle_radius[0]),
                  -angle_radius[1] * sin(angle_radius[0]));
  }
};

/////////////////////////////////////////////////////////////////////////////
// [alpha, radius] -> [x,y]
/////////////////////////////////////////////////////////////////////////////
template <typename vec2_t> class euclid_to_polar {
 public:
  vec2_t operator()(vec2_t const& euclid) {
    vec2_t angle_radius;
    angle_radius[1] = sqrt(euclid[0] * euclid[0] + euclid[1] * euclid[1]);

    if (euclid[1] < 0) {
      angle_radius[0] = acos(euclid[0] / angle_radius[1]);
    } else {
      angle_radius[0] =
          vec2_t::value_type(2) * M_PI - acos(euclid[0] / angle_radius[1]);
    }

    return angle_radius;
  }
};

}  // namespace util
}  // namespace tml

#endif  // TML_POLAR_CONVERSION_HPP
