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
#ifndef GUA_VIDEO3D_LOADER_HPP
#define GUA_VIDEO3D_LOADER_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/databases/Database.hpp>

// external headers
#include <string>
#include <list>
#include <memory>


namespace gua {

namespace node {
class Node;
class InnerNode;
class Video3DNode;
}

/**
 * Loads and draws Video3D.
 *
 * This class can load Video3D data from files and display them in multiple
 * contexts. A MeshLoader object is made of several Video3D objects.
 */
class GUA_DLL Video3DLoader : public GeometryLoader {
 public:

  Video3DLoader();
  ~Video3DLoader() = default;

  std::shared_ptr<node::Node> create_geometry_from_file(std::string const& nodename,
                                                  std::string const& ksfile);

  bool is_supported(std::string const& file_name) const override;

 private:
    boost::unordered_set<std::string> _supported_file_extensions;

};

}

#endif  // GUA_VIDEO3D_LOADER_HPP
#endif
