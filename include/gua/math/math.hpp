/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#ifndef GUA_MATH_HPP
#define GUA_MATH_HPP

#include <scm/core/math.h>
#include <scm/gl_core/math.h>

#if ASSIMP_VERSION == 3
#include <assimp/Importer.hpp>
#else
#include <assimp/assimp.hpp>
#endif

#include <gua/math/traits.hpp>

namespace gua {
namespace math {

///@{
/**
 * Some basic math types, typedef'ed from schism.
 */
typedef scm::math::mat<float, 4, 4> mat4;
typedef scm::math::mat<float, 3, 3> mat3;

typedef scm::math::vec<float, 4> vec4;
typedef scm::math::vec<float, 3> vec3;
typedef scm::math::vec<float, 2> vec2;

typedef scm::math::vec<int, 4> vec4i;
typedef scm::math::vec<int, 3> vec3i;
typedef scm::math::vec<int, 2> vec2i;

typedef scm::math::vec<unsigned, 4> vec4ui;
typedef scm::math::vec<unsigned, 3> vec3ui;
typedef scm::math::vec<unsigned, 2> vec2ui;
///@}

/**
 * Computes a frustum matrix.
 *
 * \param eye_position      World position of the camera.
 * \param screen_transform  Transformation of the unit-screen.
 * \param near_plane        Distance of the near clipping.
 * \param far_plane         Distance of the far clipping.
 *
 * \return                  A frustum matrix.
 */
math::mat4 const compute_frustum(math::vec4 const& eye_position,
                                 math::mat4 const& screen_transform,
                                 float near_plane,
                                 float far_plane);

/**
 * Converts an assimp matrix to a schism matrix.
 *
 * \param ai_mat  A assimp matrix.
 *
 * \return        A schism matrix.
 */
math::mat4 const mat_ai_to_scm(aiMatrix4x4 const& ai_mat);

#if WIN32
  template <typename T>
  inline T clamp(T const& x, T const& min, T const& max) {
    return x < min ? min : (x > max ? max : x);
  }
#else
  template <typename T>
  constexpr T clamp(T const& x, T const& min, T const& max) {
    return x < min ? min : (x > max ? max : x);
  }
#endif

inline math::vec3 get_translation(math::mat4 const& m)
{
  return math::vec3(m[12], m[13], m[14]);
}


template <typename PosType, typename ValueType>
ValueType interpolate(PosType const& position,
                      std::pair<PosType, ValueType> const& a,
                      std::pair<PosType, ValueType> const& b,
                      std::pair<PosType, ValueType> const& c) {

    // calculate vectors from position to vertices a, b and c:
    auto f1 = a.first-position;
    auto f2 = b.first-position;
    auto f3 = c.first-position;

    // calculate the areas and factors (order of parameters doesn't matter):
    auto area = scm::math::length(scm::math::cross(a.first-b.first, a.first-c.first));
    auto a1   = scm::math::length(scm::math::cross(f2, f3)) / area;
    auto a2   = scm::math::length(scm::math::cross(f3, f1)) / area;
    auto a3   = scm::math::length(scm::math::cross(f1, f2)) / area;

    return a.second * a1 + b.second * a2 + c.second * a3;
}

}
}

namespace gua {
namespace traits {

template <> struct scalar<math::vec2> {
  typedef float type;
};

template <> struct scalar<math::vec3> {
  typedef float type;
};

template <> struct scalar<math::vec4> {
  typedef float type;
};

template <> struct dimension<math::vec2> {
  static const unsigned int value = 2;
};

template <> struct dimension<math::vec3> {
  static const unsigned int value = 3;
};

template <> struct dimension<math::vec4> {
  static const unsigned int value = 4;
};

}
}

#endif // GUA_MATH_HPP
