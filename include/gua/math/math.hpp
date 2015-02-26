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
#include <tuple>

template<class TReal>
class aiMatrix4x4t;

#include <gua/platform.hpp>
#include <gua/math/traits.hpp>

namespace gua {
namespace math {

///@{
/**
 * Some basic math types, typedef'ed from schism.
 */

typedef double float_t;

// cpu types
typedef scm::math::mat<float_t, 4, 4> mat4;
typedef scm::math::mat<float_t, 3, 3> mat3;

typedef scm::math::vec<float_t, 4> vec4;
typedef scm::math::vec<float_t, 3> vec3;
typedef scm::math::vec<float_t, 2> vec2;

typedef scm::math::quat<float_t> quat;

typedef scm::math::vec<int, 4> vec4i;
typedef scm::math::vec<int, 3> vec3i;
typedef scm::math::vec<int, 2> vec2i;

typedef scm::math::vec<unsigned, 4> vec4ui;
typedef scm::math::vec<unsigned, 3> vec3ui;
typedef scm::math::vec<unsigned, 2> vec2ui;


// float types for gpu use
typedef scm::math::mat<float, 4, 4> mat4f;
typedef scm::math::mat<float, 3, 3> mat3f;

typedef scm::math::vec<float, 4> vec4f;
typedef scm::math::vec<float, 3> vec3f;
typedef scm::math::vec<float, 2> vec2f;

typedef scm::math::quat<float> quatf;

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
math::mat4 GUA_DLL compute_perspective_frustum(math::vec4 const& eye_position,
                                 math::mat4 const& screen_transform,
                                 float_t near_plane,
                                 float_t far_plane);

math::mat4 GUA_DLL compute_orthographic_frustum(math::vec4 const& eye_position,
                                 math::mat4 const& screen_transform,
                                 float_t near_plane,
                                 float_t far_plane);

/**
 * Converts an assimp matrix to a schism matrix.
 *
 * \param ai_mat  A assimp matrix.
 *
 * \return        A schism matrix.
 */
math::mat4 GUA_DLL mat_ai_to_scm(aiMatrix4x4t<float> const& ai_mat);

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

inline math::vec3 get_translation(math::mat4 const& m) {
  return math::vec3(m[12], m[13], m[14]);
}

inline math::mat4 get_rotation(math::mat4 const& m) {
  math::quat q = ::scm::math::quat<float_t>::from_matrix(m);
  return q.to_matrix();
}

std::tuple<float_t, float_t, float_t> GUA_DLL barycentric(math::vec3 const& a,
                                                          math::vec3 const& b,
                                                          math::vec3 const& c,
                                                          math::vec3 const& p);

template <typename ValueType>
ValueType interpolate(math::vec3 const& position,
                      std::pair<math::vec3, ValueType> const& a,
                      std::pair<math::vec3, ValueType> const& b,
                      std::pair<math::vec3, ValueType> const& c) {
    float u, v, w;
    std::tie(u,v,w) = barycentric(a.first,b.first,c.first,position);
    return u * a.second + v * b.second + w * c.second;
}

}
}

namespace gua {
namespace traits {

template <> struct scalar<math::vec2> {
  typedef gua::math::float_t type;
};

template <> struct scalar<math::vec3> {
  typedef gua::math::float_t type;
};

template <> struct scalar<math::vec4> {
  typedef gua::math::float_t type;
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
