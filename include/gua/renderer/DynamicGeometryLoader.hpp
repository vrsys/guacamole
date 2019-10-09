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

#ifndef GUA_DYNAMIC_GEOMETRY_LOADER_HPP
#define GUA_DYNAMIC_GEOMETRY_LOADER_HPP

// guacamole headers
#include <gua/renderer/DynamicGeometryResource.hpp>

#include <gua/renderer/Material.hpp>
#include <gua/utils/Mesh.hpp>

// external headers
#include <string>
#include <list>
#include <memory>

namespace Assimp
{
class Importer;
}
struct aiScene;
struct aiNode;

namespace gua
{
class GeometryDescription;

namespace node
{
class Node;
class InnerNode;
class GeometryNode;
class DynamicGeometryNode;
} // namespace node

/**
 * Loads and draws dynamic geometries.
 *
 * This class can load dynamic geometry data from files and display them in multiple
 * contexts. A DynamicGeometryLoader object is made of several DynamicGeometry objects.
 */
class GUA_DLL DynamicGeometryLoader
{
  public: // typedefs, enums
    enum Flags
    {
        DEFAULTS = 0,
        MAKE_PICKABLE = 1 << 2,
        NORMALIZE_POSITION = 1 << 3,
        NORMALIZE_SCALE = 1 << 4,
        NO_SHARED_MATERIALS = 1 << 5
    };

  public:
    /**
     * Default constructor.
     *
     * Constructs a new and empty MeshLoader.
     */
    DynamicGeometryLoader();

    /**
     *
     */
    std::shared_ptr<node::Node> load_geometry(std::string const& file_name, unsigned flags = DEFAULTS, bool create_empty = false);

    /**
     *
     */
    std::shared_ptr<node::Node> create_empty_geometry(std::string const& node_name, std::string const& empty_name, std::shared_ptr<Material> const& fallback_material, unsigned flags = DEFAULTS);

    /**
     *
     */
    std::shared_ptr<node::Node> create_empty_geometry(std::string const& node_name, std::string const& empty_name, unsigned flags = DEFAULTS);

    /**
     * Constructor from a file.
     *
     * Creates a new DynamicGeometryLoader from a given file.
     *
     * \param file_name        The file to load the meshs data from.
     * \param material_name    The material name that was set to the parent node
     */
    std::shared_ptr<node::Node> load(std::string const& file_name, unsigned flags, bool create_geometries, bool create_empty);

    /**
     * Constructor from memory buffer.
     *
     * Creates a new DynamicGeometryLoader from a existing memory buffer.
     *
     * \param buffer_name      The buffer to load the meh's data from.
     * \param buffer_size      The buffer's size.
     */
    std::vector<DynamicGeometryResource*> const load_from_buffer(char const* buffer_name, unsigned buffer_size, bool build_kd_tree);
    /**
     *
     */
    int is_supported(std::string const& file_name) const;

  private: // methods
    virtual std::shared_ptr<gua::node::DynamicGeometryNode> create_geometry_instance(std::shared_ptr<DynamicGeometryImporter> importer, GeometryDescription const& desc, unsigned flags) = 0;

    static void apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material, bool no_shared_materials);

  private: // attributes
    static std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>> loaded_files_;
};

} // namespace gua

#endif // GUA_DYNAMIC_GEOMETRY_LOADER_HPP
