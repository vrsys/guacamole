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

#include <gua/renderer/ProgramFactory.hpp>

#include <fstream>
#include <boost/regex.hpp>

#include <gua/config.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

ProgramFactory::ProgramFactory(std::vector<std::string> const& shader_root_directories)
    : _search_paths(shader_root_directories)
{
  add_search_path(std::string(GUACAMOLE_INSTALL_DIR));
  add_search_path(std::string(GUACAMOLE_INSTALL_DIR) + "/resources");
}

////////////////////////////////////////////////////////////////////////////////

void ProgramFactory::add_search_path(std::string const& path)
{
  _search_paths.push_back(path);
}

////////////////////////////////////////////////////////////////////////////////

std::string ProgramFactory::read_plain_file(std::string const& path) const
{
  namespace fs = boost::filesystem;
  std::string out;
  fs::path p;
  if (!get_file_contents(fs::path(path), fs::current_path(), out, p))
    throw std::runtime_error("Unable to read plain file");

  return out;
}

////////////////////////////////////////////////////////////////////////////////

std::string ProgramFactory::read_shader_file(std::string const& path) const
{
  namespace fs = boost::filesystem;

  std::string out;
  if (!resolve_includes(fs::path(path), fs::current_path(), out))
    throw std::runtime_error("Unable to resolve shader includes");

  return std::string(GUACAMOLE_GLSL_VERSION_STRING) + out;
}

////////////////////////////////////////////////////////////////////////////////

std::string ProgramFactory::resolve_substitutions(std::string const& shader_source,
                                                  SubstitutionMap const& smap) const
{
  //TODO: add support for the #line macro if multi-line substitutions are supplied.
  boost::regex regex("\\@(\\w+)\\@");
  boost::smatch match;
  std::string out, s = shader_source;

  while (boost::regex_search(s, match, regex)) {
    std::string subs;
    auto search = smap.find(match[1]);
    if (search != smap.end()) {
      subs = search->second;
    }
    else {
      Logger::LOG_WARNING << "Option \"" << match[1]
                          << "\" is unknown!" << std::endl;
      subs = match.str();
    }
    out += match.prefix().str() + subs;
    s = match.suffix().str();
  }
  return out + s;
}

////////////////////////////////////////////////////////////////////////////////

bool ProgramFactory::get_file_contents(boost::filesystem::path const& filename,
                                       boost::filesystem::path const& current_dir,
                                       std::string& contents,
                                       boost::filesystem::path& full_path) const
{
  namespace fs = boost::filesystem;
  std::ifstream ifs;
  std::stringstream error_info;

  auto probe =[&](fs::path const& dir) -> bool {
    full_path = fs::absolute(filename, dir);
    ifs.open(full_path.native());
    if (!ifs) {
      error_info << "[" << dir.native() << "]: " << strerror(errno) << std::endl;
    }
    return ifs;
  };

  // probe files
  if (!probe(current_dir)) {
    for (auto const& path : _search_paths) {
      if (probe(fs::path(path))) break;
    }
  }

  // log error if failed to find file
  if (!ifs) {
    Logger::LOG_ERROR << "Failed to get file: \"" << filename.native()
                      << "\" from any of the search paths:" << std::endl
                      << error_info.str() << std::endl;
    contents = "";
    full_path = fs::path();
    return false;
  }

  contents.assign((std::istreambuf_iterator<char>(ifs)),
                  (std::istreambuf_iterator<char>()));
  full_path = fs::canonical(full_path);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool ProgramFactory::resolve_includes(boost::filesystem::path const& filename,
                                      boost::filesystem::path const& current_dir,
                                      std::string& contents) const
{
  namespace fs = boost::filesystem;

  // get contents
  std::string s;
  fs::path full_path;
  if (!get_file_contents(filename, current_dir, s, full_path)) {
    contents = "";
    return false;
  }

  // substitute inclusions
  s = "#line 1 \"" + full_path.native() + "\"\n" + s;
  boost::regex regex("(\\@|\\#)\\s*include\\s*\"([^\"]+)\"");
  boost::smatch match;
  std::string out;
  int line_ctr{};
  while (boost::regex_search(s, match, regex)) {
    std::string content;
    if (!resolve_includes(fs::path(match[2]),
                          full_path.parent_path(),
                          content)) {
      contents = "";
      return false;
    }
    std::string prefix = match.prefix().str();
    line_ctr += std::count(prefix.begin(), prefix.end(), '\n');
    out += prefix + "\n" + content
           + "\n#line " + std::to_string(line_ctr)
           + " \"" + full_path.native() + "\"\n";
    s = match.suffix().str();
  }
  contents = out + s;
  return true;
}

////////////////////////////////////////////////////////////////////////////////

}
