/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : newton_raphson.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_NEWTON_RAPHSON_HPP
#define TML_NEWTON_RAPHSON_HPP

// header, system
#include <vector>

// header, project
#include <gua/renderer/nurbs_geometry/tml/matrix.hpp>

namespace tml {

class newton_raphson {
 public:  // typedefs

 public:  // c'tor

 public:  // method

  // volume  evaluation
  template <typename volume_type>
  inline void operator()(
      volume_type const& volume,
      typename volume_type::point_type const& target_point,
      typename volume_type::point_type const& parameter_start,
      typename volume_type::point_type& parameter_target,
      unsigned int max_iterations = 16,
      typename volume_type::value_type const& epsilon = 0.00001f) const {
    typedef typename volume_type::point_type point_type;
    typedef typename volume_type::value_type value_type;
    typedef matrix<value_type, 3, 3> matrix_type;
    assert(point_type::coordinates == 3);

    parameter_target = parameter_start;

    point_type n1 = point_type(1, 0, 0);
    point_type n2 = point_type(0, 1, 0);
    point_type n3 = point_type(0, 0, 1);

    value_type d1 = dot(-n1, target_point);
    value_type d2 = dot(-n2, target_point);
    value_type d3 = dot(-n3, target_point);

    point_type point, du, dv, dw;

    for (unsigned int i = 1; i != max_iterations; ++i) {
      volume.evaluate(parameter_target[point_type::x],
                      parameter_target[point_type::y],
                      parameter_target[point_type::z],
                      point,
                      du,
                      dv,
                      dw);

      point_type Fuvw(
          dot(point, n1) + d1, dot(point, n2) + d2, dot(point, n3) + d3);

      if (Fuvw.abs() < epsilon) {
        break;
      }

      point_type Fu(dot(n1, du), dot(n2, du), dot(n3, du));
      point_type Fv(dot(n1, dv), dot(n2, dv), dot(n3, dv));
      point_type Fw(dot(n1, dw), dot(n2, dw), dot(n3, dw));

      matrix_type J;
      J[0][0] = Fu[0];
      J[0][1] = Fv[0];
      J[0][2] = Fw[2];
      J[1][0] = Fu[1];
      J[1][1] = Fv[1];
      J[1][2] = Fw[2];
      J[2][0] = Fu[2];
      J[2][1] = Fv[2];
      J[2][2] = Fw[2];

      matrix_type Jinv = J.inverse();

      parameter_target -= Jinv * Fuvw;
    }
  }
};

}  // namespace tml

#endif  // TML_NEWTON_RAPHSON_HPP
