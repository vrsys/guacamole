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
#include <gua/databases/MaterialDatabase.hpp>

// guacamole headers
#include <gua/utils/Directory.hpp>

// external headers
#include <sstream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void MaterialDatabase::load_materials_from(
    std::string const& path_to_materials) {

  gua::Directory dir(path_to_materials);
  std::stringstream content(dir.get_content());
  std::string parse_string;

  while (content >> parse_string) {
    unsigned suffix_pos = unsigned(parse_string.find(".gmd"));

    if (parse_string.length() - suffix_pos == 4) {
      auto name(parse_string.substr(0, suffix_pos));

      auto mat = std::make_shared<Material>(
          name, MaterialDescription(dir.get_directory_name() + parse_string));

      instance()->add(name, mat);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void MaterialDatabase::reload_all() {
  for (auto const& date: data_) {
    date.second->reload();
  }
}

////////////////////////////////////////////////////////////////////////////////

}
