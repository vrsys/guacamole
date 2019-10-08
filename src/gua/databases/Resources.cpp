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

// class header
#include <gua/databases/Resources.hpp>

// guacamole headers
#include <gua/config.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <unordered_map>

namespace gua
{
namespace Resources
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
namespace
{
const std::unordered_map<std::string, std::vector<unsigned char> const*> data_;
}
#else
#include "../src/gua/generated/R.inl"
namespace
{
const std::unordered_map<std::string, std::vector<unsigned char> const*> data_(R_fill_map());
}
#endif

//////////////////////////////////////////////////////////////////////////////

void resolve_includes(std::string& shader_source)
{
    std::size_t search_pos(0);

    std::string search("@include");

    while(search_pos != std::string::npos)
    {
        // find incluse
        search_pos = shader_source.find(search, search_pos);

        if(search_pos != std::string::npos)
        {
            // get file name
            std::size_t start(shader_source.find('\"', search_pos) + 1);
            std::size_t end(shader_source.find('\"', start));

            std::string file(shader_source.substr(start, end - start));

            // get included file
            std::string include(lookup_shader(file));

            // include it
            shader_source.replace(search_pos, end - search_pos + 2, include);

            // advance search pos
            search_pos = search_pos + include.length();
        }
    }
}

//////////////////////////////////////////////////////////////////////////////

std::string lookup_string(std::string const& file)
{
    auto it(data_.find(file));

    if(it == data_.end())
        Logger::LOG_ERROR << "Failed to get string resource: Entry \"" << file << "\" does not exist!" << std::endl;

    return lookup_string(*it->second);
}

//////////////////////////////////////////////////////////////////////////////

std::string lookup_string(std::vector<unsigned char> const& resource) { return std::string(reinterpret_cast<char const*>(resource.data()), resource.size()); }

//////////////////////////////////////////////////////////////////////////////

std::string lookup_shader(std::string const& file)
{
    std::string source(lookup_string(file));

    resolve_includes(source);

    return source;
}

//////////////////////////////////////////////////////////////////////////////

std::string lookup_shader(std::vector<unsigned char> const& resource)
{
    std::string source(lookup_string(resource));

    resolve_includes(source);

    return source;
}

//////////////////////////////////////////////////////////////////////////////

std::vector<unsigned char> const& lookup(std::string const& file)
{
    auto it(data_.find(file));

    if(it == data_.end())
        Logger::LOG_ERROR << "Failed to get string resource: Entry \"" << file << "\" does not exist!" << std::endl;

    return *it->second;
}

//////////////////////////////////////////////////////////////////////////////

} // namespace Resources

} // namespace gua
