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
#if 0
// class header
#include <gua/renderer/Video3DLoader.hpp>

// guacamole headers
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/node/Video3DNode.hpp>
#include <gua/renderer/Video3DRessource.hpp>
#include <gua/renderer/Video3DUberShader.hpp>

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////

  Video3DLoader::Video3DLoader()
    : _supported_file_extensions()
  {
    _supported_file_extensions.insert("ks");
  }


  ////////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<node::Node> Video3DLoader::create_geometry_from_file (std::string const& node_name,
                                                                  std::string const& file_name)
  {
    try {
      GeometryDatabase::instance()->add(
        file_name, std::make_shared<Video3DRessource>(file_name));

      auto result = std::make_shared<node::Video3DNode>(node_name, file_name, Video3DUberShader::default_video_material_name() );
      result->update_cache();

      return result;
    }
    catch (std::exception &e) {
      Logger::LOG_WARNING << "Warning: " << e.what() << " : Failed to load Video3D object " << file_name.c_str() << std::endl;
      return nullptr;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////

  bool Video3DLoader::is_supported(std::string const& file_name) const
  {
    std::vector<std::string> filename_decomposition =
      gua::string_utils::split(file_name, '.');
    return filename_decomposition.empty()
      ? false
      : _supported_file_extensions.count(filename_decomposition.back()) > 0;
  }

}
#endif
