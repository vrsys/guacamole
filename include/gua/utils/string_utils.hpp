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

#ifndef GUA_STRING_UTILS_HPP
#define GUA_STRING_UTILS_HPP

#include <gua/platform.hpp>

#include <sstream>
#include <vector>

namespace gua
{
namespace string_utils
{
template <typename T>
inline std::string to_string(T value)
{
    std::stringstream strstr;
    strstr << value;
    return strstr.str();
}

template <typename T>
inline T from_string(std::string const value)
{
    std::stringstream strstr(value);
    T result;
    strstr >> result;
    return result;
}

GUA_DLL std::vector<std::string> split(std::string const& s, char delim);

GUA_DLL std::string& replace(std::string& str, std::string const& old_str, std::string const& new_str);

GUA_DLL std::string format_code(std::string const& code);

GUA_DLL std::string demangle_type_name(const char* name);

GUA_DLL std::string sanitize(std::string const& str);

} // namespace string_utils
} // namespace gua

#endif // GUA_STRING_UTILS_HPP
