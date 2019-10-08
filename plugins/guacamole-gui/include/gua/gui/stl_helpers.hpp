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

#ifndef GUA_GUI_STL_HELPERS_HPP
#define GUA_GUI_STL_HELPERS_HPP

// includes  -------------------------------------------------------------------
#include <vector>
#include <iostream>
#include <sstream>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

template <typename T>
T clamp(T val, T a, T b)
{
    return std::min(std::max(val, a < b ? a : b), a < b ? b : a);
}

////////////////////////////////////////////////////////////////////////////////

template <typename T>
T from_string(std::string const& v)
{
    std::istringstream iss(v);
    T result;
    iss >> result;
    return result;
}

////////////////////////////////////////////////////////////////////////////////

template <typename T>
std::string to_string(T const& v)
{
    std::ostringstream oss;
    oss << v;
    return oss.str();
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua

namespace std
{
////////////////////////////////////////////////////////////////////////////////

template <typename T>
std::ostream& operator<<(std::ostream& os, std::vector<T> const& v)
{
    typename std::vector<T>::const_iterator i(v.begin());
    while(i != v.end())
    {
        os << *i;

        if(++i != v.end())
        {
            os << " ";
        }
    }
    return os;
}

////////////////////////////////////////////////////////////////////////////////

template <typename T>
std::istream& operator>>(std::istream& is, std::vector<T>& v)
{
    v.clear();

    T new_one;
    while(is >> new_one)
    {
        v.push_back(new_one);
    }

    is.clear();

    return is;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace std
#endif // GUA_GUI_STL_HELPERS_HPP
