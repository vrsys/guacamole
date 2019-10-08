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

#ifndef GUA_PROGRAM_FACTORY_HPP
#define GUA_PROGRAM_FACTORY_HPP

#include <vector>
#include <map>
#include <unordered_map>
#include <list>
#include <boost/filesystem.hpp>

#include <gua/platform.hpp>

namespace gua
{
using SubstitutionMap = std::unordered_map<std::string, std::string>;

class GUA_DLL ResourceFactory
{
  public:
    ResourceFactory(std::vector<std::string> const& search_directories = std::vector<std::string>());

    virtual ~ResourceFactory() {}

    void add_search_path(std::string const& path);

    std::string read_plain_file(std::string const& file) const;
    std::string read_shader_file(std::string const& file) const;
    std::string prepare_shader(std::string const& shader_source, std::string const& label) const;
    std::string resolve_substitutions(std::string const& shader_source, SubstitutionMap const& smap) const;

  private:
    bool get_file_contents(boost::filesystem::path const& filename, boost::filesystem::path const& current_dir, std::string& contents, boost::filesystem::path& full_path) const;

    bool get_file_contents(boost::filesystem::path const& filename, boost::filesystem::path const& current_dir, std::wstring& contents, boost::filesystem::path& full_path) const;

    bool resolve_includes(boost::filesystem::path const& filename, boost::filesystem::path const& current_dir, std::string& contents, std::string const& custom_label = std::string()) const;

    std::vector<std::string> _search_paths;
};

} // namespace gua

#endif // GUA_PIPELINE_HPP
