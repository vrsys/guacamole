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

#include <gua/utils/string_utils.hpp>

#include <algorithm>

#ifdef __GNUG__
#include <memory>
#include <cxxabi.h>
#endif

namespace gua
{
namespace string_utils
{
////////////////////////////////////////////////////////////////////////////

std::vector<std::string> split(std::string const& s, char delim)
{
    std::vector<std::string> elems;

    std::stringstream ss(s);
    std::string item;

    while(std::getline(ss, item, delim))
    {
        elems.push_back(item);
    }

    return elems;
}

////////////////////////////////////////////////////////////////////////////

std::string& replace(std::string& str, std::string const& old_str, std::string const& new_str)
{
    std::size_t pos = 0;
    while((pos = str.find(old_str, pos)) != std::string::npos)
    {
        str.replace(pos, old_str.length(), new_str);
        pos += new_str.length();
    }
    return str;
}

////////////////////////////////////////////////////////////////////////////

std::string format_code(std::string const& code)
{
    std::string result(code);

    for(int i(0); i < 20; ++i)
        replace(result, "  ", " ");

    replace(result, " \n", "\n");
    replace(result, "\n ", "\n");

    int depth(0);

    for(unsigned pos(0); pos < result.length(); ++pos)
    {
        if(result[pos] == '{')
        {
            ++depth;
        }

        if(pos + 1 < result.length() && result[pos + 1] == '}')
        {
            --depth;
        }

        if(result[pos] == '\n' && depth > 0)
        {
            result.insert(pos + 1, depth * 4, ' ');
            pos += 4;
        }
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////

std::string demangle_type_name(const char* name)
{
#ifdef __GNUG__
    // implementation taken from http://stackoverflow.com/a/4541470
    int status = -4;
    std::unique_ptr<char, void (*)(void*)> res{abi::__cxa_demangle(name, nullptr, nullptr, &status), std::free};
    return (status == 0) ? res.get() : name;
#else
    return name;
#endif
}

////////////////////////////////////////////////////////////////////////////

std::string sanitize(std::string const& str)
{
    std::string s(str);
    std::string illegal(" |\\?*+\":<>[](){}/'.,#&\r\n\t");

    std::replace_if(s.begin(), s.end(), [&illegal](char const c) { return illegal.find_first_of(c) != std::string::npos; }, '_');

    return s;
}

////////////////////////////////////////////////////////////////////////////

} // namespace string_utils
} // namespace gua
