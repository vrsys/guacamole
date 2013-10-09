/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : rapid.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_RAPID_HPP
#define TML_RAPID_HPP

// header, system
#include <cmath>
#include <iostream>

#include <boost/array.hpp>

#include <gua/renderer/nurbs_geometry/tml/matrix.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

namespace tml {

template <typename value_t>
int obb_disjoint(matrix<value_t, 3, 3> const& B,
                 point<value_t, 3> const& T,
                 point<value_t, 3> const& a,
                 point<value_t, 3> const& b) {
  register value_t t, s;
  register int r;
  value_t Bf[3][3];
  const value_t reps = value_t(1e-6);

  // Bf = fabs(B)
  Bf[0][0] = fabs(B[0][0]);
  Bf[0][0] += reps;
  Bf[0][1] = fabs(B[0][1]);
  Bf[0][1] += reps;
  Bf[0][2] = fabs(B[0][2]);
  Bf[0][2] += reps;
  Bf[1][0] = fabs(B[1][0]);
  Bf[1][0] += reps;
  Bf[1][1] = fabs(B[1][1]);
  Bf[1][1] += reps;
  Bf[1][2] = fabs(B[1][2]);
  Bf[1][2] += reps;
  Bf[2][0] = fabs(B[2][0]);
  Bf[2][0] += reps;
  Bf[2][1] = fabs(B[2][1]);
  Bf[2][1] += reps;
  Bf[2][2] = fabs(B[2][2]);
  Bf[2][2] += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint
  r = 1;

  // A1 x A2 = A0
  t = fabs(T[0]);

  r &= (t <= (a[0] + b[0] * Bf[0][0] + b[1] * Bf[0][1] + b[2] * Bf[0][2]));
  if (!r)
    return 1;

  // B1 x B2 = B0
  s = T[0] * B[0][0] + T[1] * B[1][0] + T[2] * B[2][0];
  t = fabs(s);

  r &= (t <= (b[0] + a[0] * Bf[0][0] + a[1] * Bf[1][0] + a[2] * Bf[2][0]));
  if (!r)
    return 2;

  // A2 x A0 = A1
  t = fabs(T[1]);

  r &= (t <= (a[1] + b[0] * Bf[1][0] + b[1] * Bf[1][1] + b[2] * Bf[1][2]));
  if (!r)
    return 3;

  // A0 x A1 = A2
  t = fabs(T[2]);

  r &= (t <= (a[2] + b[0] * Bf[2][0] + b[1] * Bf[2][1] + b[2] * Bf[2][2]));
  if (!r)
    return 4;

  // B2 x B0 = B1
  s = T[0] * B[0][1] + T[1] * B[1][1] + T[2] * B[2][1];
  t = fabs(s);

  r &= (t <= (b[1] + a[0] * Bf[0][1] + a[1] * Bf[1][1] + a[2] * Bf[2][1]));
  if (!r)
    return 5;

  // B0 x B1 = B2
  s = T[0] * B[0][2] + T[1] * B[1][2] + T[2] * B[2][2];
  t = fabs(s);

  r &= (t <= (b[2] + a[0] * Bf[0][2] + a[1] * Bf[1][2] + a[2] * Bf[2][2]));
  if (!r)
    return 6;

  // A0 x B0
  s = T[2] * B[1][0] - T[1] * B[2][0];
  t = fabs(s);

  r &= (t <= (a[1] * Bf[2][0] + a[2] * Bf[1][0] + b[1] * Bf[0][2] +
              b[2] * Bf[0][1]));
  if (!r)
    return 7;

  // A0 x B1
  s = T[2] * B[1][1] - T[1] * B[2][1];
  t = fabs(s);

  r &= (t <= (a[1] * Bf[2][1] + a[2] * Bf[1][1] + b[0] * Bf[0][2] +
              b[2] * Bf[0][0]));
  if (!r)
    return 8;

  // A0 x B2
  s = T[2] * B[1][2] - T[1] * B[2][2];
  t = fabs(s);

  r &= (t <= (a[1] * Bf[2][2] + a[2] * Bf[1][2] + b[0] * Bf[0][1] +
              b[1] * Bf[0][0]));
  if (!r)
    return 9;

  // A1 x B0
  s = T[0] * B[2][0] - T[2] * B[0][0];
  t = fabs(s);

  r &= (t <= (a[0] * Bf[2][0] + a[2] * Bf[0][0] + b[1] * Bf[1][2] +
              b[2] * Bf[1][1]));
  if (!r)
    return 10;

  // A1 x B1
  s = T[0] * B[2][1] - T[2] * B[0][1];
  t = fabs(s);

  r &= (t <= (a[0] * Bf[2][1] + a[2] * Bf[0][1] + b[0] * Bf[1][2] +
              b[2] * Bf[1][0]));
  if (!r)
    return 11;

  // A1 x B2
  s = T[0] * B[2][2] - T[2] * B[0][2];
  t = fabs(s);

  r &= (t <= (a[0] * Bf[2][2] + a[2] * Bf[0][2] + b[0] * Bf[1][1] +
              b[1] * Bf[1][0]));
  if (!r)
    return 12;

  // A2 x B0
  s = T[1] * B[0][0] - T[0] * B[1][0];
  t = fabs(s);

  r &= (t <= (a[0] * Bf[1][0] + a[1] * Bf[0][0] + b[1] * Bf[2][2] +
              b[2] * Bf[2][1]));
  if (!r)
    return 13;

  // A2 x B1
  s = T[1] * B[0][1] - T[0] * B[1][1];
  t = fabs(s);

  r &= (t <= (a[0] * Bf[1][1] + a[1] * Bf[0][1] + b[0] * Bf[2][2] +
              b[2] * Bf[2][0]));
  if (!r)
    return 14;

  // A2 x B2
  s = T[1] * B[0][2] - T[0] * B[1][2];
  t = fabs(s);

  r &= (t <= (a[0] * Bf[1][2] + a[1] * Bf[0][2] + b[0] * Bf[2][1] +
              b[1] * Bf[2][0]));
  if (!r)
    return 15;

  return 0;  // should equal 0
}

}  // namespace tml

#endif  // TML_RAPID_HPP
