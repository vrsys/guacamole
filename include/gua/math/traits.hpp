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

#ifndef GUA_TRAITS_HPP
#define GUA_TRAITS_HPP

#include <boost/mpl/assert.hpp>

namespace gua
{
namespace traits
{
// vector space meta functions

template <typename T>
struct scalar
{
    BOOST_MPL_ASSERT_MSG(false, NOT_IMPLEMENTED_FOR_THIS_VECTOR_TYPE, (types<T>));
};

template <>
struct scalar<float>
{
    using type = float;
};
template <>
struct scalar<double>
{
    using type = double;
};

template <>
struct scalar<long double>
{
    using type = long double;
};

template <typename T>
struct dimension
{
    BOOST_MPL_ASSERT_MSG(false, NOT_IMPLEMENTED_FOR_THIS_VECTOR_TYPE, (types<T>));
};

template <>
struct dimension<float>
{
    static const unsigned int value = 1;
};
template <>
struct dimension<double>
{
    static const unsigned int value = 1;
};
template <>
struct dimension<long double>
{
    static const unsigned int value = 1;
};

// Examples:
//
// template<>
// struct scalar<glm::vec3>
// {
//   using type = glm::vec3::value_type;
// };
//
// template<>
// struct dimension<glm::vec3>
// {
//   static const unsigned int value = 3;
// };
//
// template<T,N>
// struct scalar< ::scm::math::vec<T, N> >
// {
//   using type = typename ::scm::math::vec<T, N>::value_type;
// };
//
// template<T,N>
// struct dimension< ::scm::math::vec<T, N> >
// {
//   static const unsigned int value = N;
// };
} // namespace traits
} // namespace gua

#endif // #ifndef GUA_TRAITS_HPP
