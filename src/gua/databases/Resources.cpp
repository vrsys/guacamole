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
#include <gua/utils/logger.hpp>

// external headers
#include <unordered_map>

namespace gua {

namespace Resources {

  #include "../src/gua/generated/R.inl"

  //////////////////////////////////////////////////////////////////////////////

  namespace {
    const std::unordered_map<std::string, std::vector<unsigned char> const*> data_(R_fill_map());

    void resolve_includes(std::string& shader_source) {
      int search_pos(0);

      std::string search("@include");

      while(search_pos != std::string::npos) {
        // find incluse
        search_pos = shader_source.find(search, search_pos);

        if (search_pos != std::string::npos) {

          // get file name
          int start(shader_source.find('\"', search_pos)+1);
          int end  (shader_source.find('\"', start));

          std::string file(shader_source.substr(start, end-start));

          // get included file
          std::string include(lookup_shader(file));

          // include it
          shader_source.replace(search_pos, end-search_pos + 2, include);

          // advance search pos
          search_pos = search_pos + include.length();
        }
      }
    }

  }

  //////////////////////////////////////////////////////////////////////////////

  std::string lookup_string(std::string const& file) {
    auto it(data_.find(file));

    if (it == data_.end())
      ERROR("Failed to get string resource: Entry \"%s\" does not exist!", file.c_str());

    return lookup_string(*it->second);
  }

  //////////////////////////////////////////////////////////////////////////////

  std::string lookup_string(std::vector<unsigned char> const& resource) {
    return std::string(reinterpret_cast<char const*>(&resource[0]), resource.size());
  }

  //////////////////////////////////////////////////////////////////////////////

  std::string lookup_shader(std::string const& file) {
    std::string source(lookup_string(file));

    resolve_includes(source);

    return source;
  }

  //////////////////////////////////////////////////////////////////////////////

  std::string lookup_shader(std::vector<unsigned char> const& resource) {
    std::string source(lookup_string(resource));

    resolve_includes(source);

    return source;
  }

  //////////////////////////////////////////////////////////////////////////////

  std::vector<unsigned char> const& lookup(std::string const& file) {
    auto it(data_.find(file));

    if (it == data_.end())
      ERROR("Failed to get string resource: Entry \"%s\" does not exist!", file.c_str());

    return *it->second;
  }

  //////////////////////////////////////////////////////////////////////////////

}

}
