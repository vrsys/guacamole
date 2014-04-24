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
#include <gua/renderer/Video3DLoader.hpp>

// guacamole headers
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/scenegraph/Video3DNode.hpp>
#include <gua/renderer/Video3D.hpp>

namespace gua {
  
Video3DLoader::Video3DLoader() : LoaderBase(), _supported_file_extensions() {
  _supported_file_extensions.insert("ks");    
}


std::shared_ptr<Node> Video3DLoader::load(std::string const& file_name,
                                       unsigned flags) {
  try {
      GeometryDatabase::instance()->add(
        file_name, std::make_shared<Video3D>(file_name));

      auto result = std::make_shared<Video3DNode>("unnamed_video3D");
      result->set_ksfile(file_name);
      result->set_material("");     

      return result;

    }
    catch (std::exception &e) {
      Logger::LOG_WARNING << "Warning: " << e.what() << " : Failed to load Video3D object " << file_name.c_str() << std::endl;
      return nullptr;
    }
}

  bool Video3DLoader::is_supported(std::string const& file_name) const 
  {
    std::vector<std::string> filename_decomposition =
      gua::string_utils::split(file_name, '.');
    return filename_decomposition.empty()
      ? false
      : _supported_file_extensions.count(filename_decomposition.back()) > 0;
  }


}
