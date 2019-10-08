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
#include <gua/video3d/platform.hpp>
#include <gua/databases/Database.hpp>

// external headers
#include <string>
#include <list>
#include <memory>

namespace gua
{
class Material;

namespace node
{
class Node;
class InnerNode;
class Video3DNode;
} // namespace node

/**
 * Loads and draws Video3D.
 *
 * This class can load Video3D data from files and display them in multiple
 * contexts. A MeshLoader object is made of several Video3D objects.
 */
class GUA_VIDEO3D_DLL Video3DLoader
{
  public:
    enum Flags
    {
        DEFAULTS = 0,
        MAKE_PICKABLE = 1 << 0,
        NORMALIZE_POSITION = 1 << 1,
        NORMALIZE_SCALE = 1 << 2
    };

    Video3DLoader();
    ~Video3DLoader() = default;

    std::shared_ptr<node::Video3DNode> create_geometry_from_file(std::string const& nodename, std::string const& ksfile, std::shared_ptr<Material> material = nullptr, unsigned flags = DEFAULTS);

  private:
};

} // namespace gua

#endif // GUA_VIDEO3D_LOADER_HPP
