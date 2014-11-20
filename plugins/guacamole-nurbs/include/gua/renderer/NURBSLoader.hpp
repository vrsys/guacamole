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
#ifndef GUA_NURBS_LOADER_HPP
#define GUA_NURBS_LOADER_HPP

// guacamole headers
#include <gua/platform.hpp>

// external headers
#include <unordered_set>

namespace gua {

namespace node {
class Node;
}

/**
 * Loads NURBS files and creates NURBS nodes.
 *
 * This class can load NURBS data from files and display them in multiple
 * contexts.
 */
class GUA_NURBS_DLL NURBSLoader {
 public:

   enum Flags {
     DEFAULTS = 0,
     MAKE_PICKABLE = 1 << 0,
     NORMALIZE_POSITION = 1 << 1,
     NORMALIZE_SCALE = 1 << 2,
     WIREFRAME = 1 << 3,
     RAYCASTING = 1 << 4
   };

  /**
   * Default constructor.
   *
   * Constructs a new and empty NURBSLoader.
   */
  NURBSLoader();

  std::shared_ptr<node::NURBSNode> create_geometry_from_file(std::string const& nodename,
                                                             std::string const& filename,
                                                             std::string const& material,
                                                             unsigned flags);

 private:

  bool is_supported(std::string const& file_name) const;

 private:

  std::unordered_set<std::string> _supported_file_extensions;

};

}

#endif  // GUA_NURBS_LOADER_HPP
