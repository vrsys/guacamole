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

#ifndef GUA_VIDEO3D_LOADER_HPP
#define GUA_VIDEO3D_LOADER_HPP

// guacamole headers
#include <gua/renderer/LoaderBase.hpp>
#include <gua/databases/Database.hpp>

// external headers
#include <string>
#include <list>
#include <memory>


namespace gua {

class Node;
class InnerNode;
class Video3DNode;

/**
 * Loads and draws Video3D.
 *
 * This class can load Video3D data from files and display them in multiple
 * contexts. A MeshLoader object is made of several Video3D objects.
 */
class Video3DLoader : public LoaderBase { //GUA_DLL??? siehe VolumeLoader
 public:

  /**
   * Default constructor.
   *
   * Constructs a new and empty Video3D.
   */
  Video3DLoader();

  /**
   * Constructor from a file.
   *
   * Creates a new Video3D from a given file.
   *
   * \param file_name        The file to load the Video3Ds data from.
   * \param material_name    The material name that was set to the parent node
   */
  std::shared_ptr<Node> load(std::string const& file_name,
                             unsigned flags);

  bool is_supported(std::string const& file_name) const;

 private:

};

}

#endif  // GUA_VIDEO3D_LOADER_HPP
