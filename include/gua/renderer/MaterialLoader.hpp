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

#ifndef GUA_MATERIAL_LOADER_HPP
#define GUA_MATERIAL_LOADER_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/utils/fbxfwd.hpp>

// external headers
#include <string>
#include <memory>

namespace Assimp
{
class Importer;
}
struct aiMaterial;

namespace gua
{
class Node;
class GeometryNode;

/**
 * Loads and draws meshes.
 *
 * This class can load mesh data from files and display them in multiple
 * contexts. A MaterialLoader object is made of several Mesh objects.
 */
class GUA_DLL MaterialLoader
{
  public:
    std::shared_ptr<Material> load_material(aiMaterial const* material, std::string const& assets_directory, bool optimize_material = true, bool nrp = false) const;
#ifdef GUACAMOLE_FBX
    std::shared_ptr<Material> load_material(FbxSurfaceMaterial const& material, std::string const& assets_directory, bool optimize_material = true, bool nrp = false) const;

    std::shared_ptr<Material> load_unreal(std::string const& file_name, std::string const& assets_directory, bool optimize_material = true, bool nrp = false) const;
#endif
    std::shared_ptr<Material> load_material(std::string const& file_name, std::string const& assets_directory, bool optimize_material = true, bool nrp = false) const;

    static std::string get_file_name(std::string const& path);
    inline static bool file_exists(std::string const& path);
};

} // namespace gua

#endif // GUA_MATERIAL_LOADER_HPP
