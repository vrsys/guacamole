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
#include <cmath>
#include <tuple>

template <class TReal>
class aiMatrix4x4t;

#include <gua/platform.hpp>
#include <gua/math/traits.hpp>

namespace gua
{
namespace math
{
///@{
/**
 * Some basic math types, typedef'ed from schism.
 */

using float_t = double;

// cpu types
template <typename T>
using M44 = scm::math::mat<T, 4, 4>;
template <typename T>
using M33 = scm::math::mat<T, 3, 3>;
template <typename T>
using M22 = scm::math::mat<T, 2, 2>;

template <typename T>
using V4 = scm::math::vec<T, 4>;
template <typename T>
using V3 = scm::math::vec<T, 3>;
template <typename T>
using V2 = scm::math::vec<T, 2>;
template <typename T>
using V1 = T;

using mat4 = M44<float_t>;
using mat4d = M44<double>;
using mat4f = M44<float>;

using mat3 = M33<float_t>;
using mat3d = M33<double>;
using mat3f = M33<float>;

using mat2 = M22<float_t>;
using mat2d = M22<double>;
using mat2f = M22<float>;

using vec4 = V4<float_t>;
using vec4d = V4<double>;
using vec4f = V4<float>;
using vec4i = V4<int>;
using vec4ui = V4<unsigned>;

using vec3 = V3<float_t>;
using vec3d = V3<double>;
using vec3f = V3<float>;
using vec3i = V3<int>;
using vec3ui = V3<unsigned>;

using vec2 = V2<float_t>;
using vec2d = V2<double>;
using vec2f = V2<float>;
using vec2i = V2<int>;
using vec2ui = V2<unsigned>;

using quat = scm::math::quat<float_t>;
using quatd = scm::math::quat<double>;
using quatf = scm::math::quat<float>;

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
math::mat4 GUA_DLL compute_perspective_frustum(math::vec4 const& eye_position, math::mat4 const& screen_transform, math::mat4::value_type near_plane, math::mat4::value_type far_plane);

math::mat4 GUA_DLL compute_orthographic_frustum(math::vec4 const& eye_position, math::mat4 const& screen_transform, math::mat4::value_type near_plane, math::mat4::value_type far_plane);

/**
 * Converts an assimp matrix to a schism matrix.
 *
 * \param ai_mat  An assimp matrix.
 *
 * \return        A schism matrix.
 */
math::mat4 GUA_DLL mat_ai_to_scm(aiMatrix4x4t<float> const& ai_mat);

#if WIN32
template <typename T>
inline T clamp(T const& x, T const& min, T const& max)
{
    return x < min ? min : (x > max ? max : x);
}
#else
template <typename T>
constexpr T clamp(T const& x, T const& min, T const& max)
{
    return x < min ? min : (x > max ? max : x);
}
#endif

inline math::vec3 get_translation(math::mat4 const& m) { return math::vec3(m[12], m[13], m[14]); }

inline math::mat4 get_rotation(math::mat4 const& m)
{
    auto q = ::scm::math::quat<math::mat4d::value_type>::from_matrix(m);
    return q.to_matrix();
}

inline math::mat4 get_yaw_matrix(math::mat4 const& m)
{
	math::mat4 forward_matrix = m * scm::math::make_translation(0.0, 0.0, -1.0);
	math::vec4 m_trans = math::vec3(m.column(3));
	math::vec4 forward_trans = math::vec3(forward_matrix.column(3));

	math::vec3 diff = math::vec3(forward_trans.x - m_trans.x, 0.0, 
		                         forward_trans.z - m_trans.z);
	
	double yaw = (atan2(-1.0, 0.0) - atan2(diff.z, diff.x)) * 180.0/3.14159265;
	return scm::math::make_translation(m_trans.x, m_trans.y, m_trans.z) * scm::math::make_rotation(yaw, 0.0, 1.0, 0.0);
}

inline math::vec4 get_scale(math::mat4 const& m)
{
    math::vec3 x_vec(m[0], m[1], m[2]);
    math::vec3 y_vec(m[4], m[5], m[6]);
    math::vec3 z_vec(m[8], m[9], m[10]);
    return math::vec3(scm::math::length(x_vec), scm::math::length(y_vec), scm::math::length(z_vec));
}

std::tuple<float_t, float_t, float_t> GUA_DLL barycentric(math::vec3 const& a, math::vec3 const& b, math::vec3 const& c, math::vec3 const& p);

template <typename ValueType>
ValueType interpolate(math::vec3 const& position, std::pair<math::vec3, ValueType> const& a, std::pair<math::vec3, ValueType> const& b, std::pair<math::vec3, ValueType> const& c)
{
    float u, v, w;
    std::tie(u, v, w) = barycentric(a.first, b.first, c.first, position);
    return u * a.second + v * b.second + w * c.second;
}
} // namespace math
} // namespace gua

namespace gua
{
namespace traits
{
template <>
struct scalar<math::vec2d>
{
    using type = math::vec2d::value_type;
};

template <>
struct scalar<math::vec3d>
{
    using type = math::vec3d::value_type;
};

template <>
struct scalar<math::vec4d>
{
    using type = math::vec4d::value_type;
};

template <>
struct dimension<math::vec2d>
{
    static const unsigned int value = 2;
};

template <>
struct dimension<math::vec3d>
{
    static const unsigned int value = 3;
};

template <>
struct dimension<math::vec4d>
{
    static const unsigned int value = 4;
};
} // namespace traits
} // namespace gua

#endif // GUA_MATH_HPP
